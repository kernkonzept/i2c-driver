#include <l4/sys/factory>
#include <l4/sys/cxx/ipc_types>
#include <l4/sys/cxx/ipc_epiface>
#include <l4/re/util/object_registry>
#include <l4/re/util/br_manager>
#include <l4/re/util/debug>

#include <l4/i2c-driver/i2c_server.h>
#include <l4/i2c-driver/i2c_device_if.h>
#include <l4/i2c-driver/i2c_controller_if.h>

#include <pthread-l4.h>
#include <memory>
#include <tuple>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cstring>
#include <string>

using Dbg = L4Re::Util::Dbg;
L4Re::Util::Err err(L4Re::Util::Err::Normal, "I2C Server");


static bool
parse_uint_optstring(char const *optstring, unsigned long *out)
{
  char *endp;

  errno = 0;
  unsigned long num = strtoul(optstring, &endp, 16);

  // check that long can be converted to int
  if (errno || *endp != '\0' || num > ULONG_MAX)
    return false;

  *out = num;

  return true;
}

static bool
parse_uint_param(L4::Ipc::Varg const &param, char const *prefix,
                 unsigned long *out)
{
  l4_size_t headlen = strlen(prefix);

  if (param.length() < headlen)
    return false;

  char const *pstr = param.value<char const *>();

  if (strncmp(pstr, prefix, headlen) != 0)
    return false;

  std::string tail(pstr + headlen, param.length() - headlen);

  if (!parse_uint_optstring(tail.c_str(), out))
    {
      err.printf("Bad paramter '%s'. Invalid number specified.\n", prefix);
      throw L4::Runtime_error(-L4_EINVAL);
    }

  return true;
}

class I2c_device : public L4::Epiface_t<I2c_device, I2c_device_ops>
{
public:
  I2c_device(l4_uint16_t dev_addr, Controller_if *ctrl)
  : _ctrl(ctrl), _addr(dev_addr)
  {}
  long op_read(I2c_device_ops::Rights, L4::Ipc::Array_ref<l4_uint8_t> &buf);
  long op_write(I2c_device_ops::Rights,
                L4::Ipc::Array_ref<l4_uint8_t const> buf);

private:
  Controller_if *_ctrl;
  l4_uint16_t _addr; ///< 7-bit or 10-bit HW device address
};

long
I2c_device::op_read(I2c_device_ops::Rights, L4::Ipc::Array_ref<l4_uint8_t> &buf)
{
  if (!_ctrl)
    return -L4_ENOSYS;

  std::vector<l4_uint8_t> buffer(buf.length);
  long err = _ctrl->read(_addr, &buffer.front(), buf.length);

  if (err >= 0)
    {
      memcpy(buf.data, &buffer.front(), buf.length);
      return L4_EOK;
    }
  else
    return err;
}

long
I2c_device::op_write(I2c_device_ops::Rights,
                     L4::Ipc::Array_ref<l4_uint8_t const> /*buf*/)
{
  return -L4_ENOSYS;
}


template <typename SRV>
class I2c_factory : public L4::Epiface_t<I2c_factory<SRV>, L4::Factory>
{
public:
  I2c_factory(std::shared_ptr<SRV> server, Controller_if *ctrl)
  : _ctrl(ctrl), _server(server)
  {}

  /**
   * Establish a connection to an i2c device.
   *
   * \param[out] res   Capability to access device.
   * \param      type  Object protocol: 0
   * \param      args  Arguments for device creation in order: i2c device address
   */
  long op_create(L4::Factory::Rights, L4::Ipc::Cap<void> &res,
                 l4_umword_t type, L4::Ipc::Varg_list<> &&args);

private:
  bool device_address_exists(unsigned dev_addr);

  Controller_if *_ctrl;
  std::shared_ptr<SRV> _server;
  std::vector<std::unique_ptr<I2c_device>> _devices;
};

template <typename SRV>
bool
I2c_factory<SRV>::device_address_exists(unsigned dev_addr)
{
  l4_uint8_t buf[1] = {0x0};
  long ret = _ctrl->write(dev_addr, buf, 1U);
  return ret != -L4_ENODEV || ret >= 0;
}

template <typename SRV>
long
I2c_factory<SRV>::op_create(L4::Factory::Rights, L4::Ipc::Cap<void> &res,
                            l4_umword_t type, L4::Ipc::Varg_list<> &&args)
{
  printf("Received create request for type %lu\n", type);
  if (type != 0)
    return -L4_ENODEV;

  long unsigned dev_addr = 0;
  for (L4::Ipc::Varg const &arg : args)
    {
      if (!arg.is_of<char const *>())
        {
          err.printf("Unexpected type for argument\n");
          return -L4_EINVAL;
        }

      try
        {
          if (parse_uint_param(arg, "addr=", &dev_addr))
            {
              if (dev_addr >= (1 << 7))
                {
                  err.printf("Requested device address %lu exceeds 7-bit "
                             "address range\n", dev_addr);
                  return -L4_EINVAL;
                }
              continue;
            }
        }
      catch (L4::Runtime_error &e)
        {
          return e.err_no();
        }
    }

  if (!device_address_exists(dev_addr))
    return -L4_ENODEV;

  std::unique_ptr<I2c_device> i2c_dev = std::make_unique<I2c_device>(dev_addr, _ctrl);
  auto device = _server->registry()->register_obj(i2c_dev.get());
  if (!device)
    return -L4_EINVAL;

  printf("Created device for addr %lu: %p\n", dev_addr, i2c_dev.get());
  res = L4::Ipc::make_cap_rw(device);
  _devices.push_back(std::move(i2c_dev));

  return L4_EOK;
}

template <typename SRV>
static std::tuple<L4::Cap<void>, std::unique_ptr<I2c_factory<SRV>>>
init_factory(std::shared_ptr<SRV> server, char const *cap_name,
             Controller_if *ctrl)
{
  std::unique_ptr<I2c_factory<SRV>> factory =
    std::make_unique<I2c_factory<SRV>>(server, ctrl);

  L4::Cap<void> cap = server->registry()->register_obj(factory.get(), cap_name);

  if (!cap)
    {
      Dbg().printf("Capability %s for factory interface not in capability table. No i2c devices can be connected.\n",
                   cap_name);
      return std::make_tuple(cap, nullptr);
    }
  else
    printf("factory cap 0x%lx\n", factory->obj_cap().cap());


  return std::make_tuple(cap, std::move(factory));
}

void start_server(Controller_if *ctrl)
{
  using Reg_Server =
    L4Re::Util::Registry_server<L4Re::Util::Br_manager_timeout_hooks>;
  // use myself as server thread.
  auto server = std::make_shared<Reg_Server>(Pthread::L4::cap(pthread_self()),
                                             L4Re::Env::env()->factory());

  ctrl->setup(server->registry());

  auto [factory_cap, factory] = init_factory(server, "dev_factory", ctrl);

  if (factory != nullptr)
    {
      printf("factory initialized. start server loop\n");
      // start server loop to allow for factory connections.
      server->loop();

      printf("exited server loop. Cleaning up ...\n");
      server->registry()->unregister_obj(factory.get());
      factory_cap.invalidate();
    }
  else
    printf("Factory registration failed.\n");;
}
