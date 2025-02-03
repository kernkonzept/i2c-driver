/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Philipp Eppelt philipp.eppelt@kernkonzept.com
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

#include <l4/sys/factory>
#include <l4/sys/cxx/ipc_types>
#include <l4/sys/cxx/ipc_epiface>
#include <l4/re/util/object_registry>
#include <l4/re/util/br_manager>
#include <l4/l4virtio/server/virtio-i2c-device>

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
#include <cassert>

#include "debug.h"

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
  {
    assert(_ctrl);
  }
  long op_read(I2c_device_ops::Rights, L4::Ipc::Array_ref<l4_uint8_t> &buf);
  long op_write(I2c_device_ops::Rights,
                L4::Ipc::Array_ref<l4_uint8_t const> buf);

  bool match(l4_uint16_t addr) const { return addr == _addr; }

private:
  Controller_if *_ctrl;
  l4_uint16_t _addr; ///< 7-bit or 10-bit HW device address
};

long
I2c_device::op_read(I2c_device_ops::Rights, L4::Ipc::Array_ref<l4_uint8_t> &buf)
{
  std::vector<l4_uint8_t> buffer(buf.length);
  long err = _ctrl->read(_addr, &buffer.front(), buf.length);

  if (err >= 0)
    {
      memcpy(buf.data, buffer.data(), buf.length);
      return L4_EOK;
    }
  else
    return err;
}

long
I2c_device::op_write(I2c_device_ops::Rights,
                     L4::Ipc::Array_ref<l4_uint8_t const> buf)
{
  std::vector<l4_uint8_t> buffer(buf.length);
  memcpy(buffer.data(), buf.data, buf.length);

  long err = _ctrl->write(_addr, &buffer.front(), buf.length);
  return err;
}

class I2c_virtio_request_handler
{
public:
  I2c_virtio_request_handler(Controller_if *ctrl, l4_uint16_t dev_addr)
  : _ctrl(ctrl), _addr(dev_addr)
  {}

  ~I2c_virtio_request_handler() = default;

  bool handle_read(l4_uint8_t *buf, unsigned len)
  {
    long err = _ctrl->read(_addr, buf, len);
    if (err)
      warn().printf("i2c-virtio-req::read: %li\n", err);
    return err == L4_EOK;
  }

  bool handle_write(l4_uint8_t const *buf, unsigned len)
  {
    long err = _ctrl->write(_addr, buf, len);
    if (err)
      warn().printf("i2c-virtio-req::write: %li\n", err);
    return err == L4_EOK;
  }

  bool match(l4_uint16_t addr) const { return addr == _addr; }

private:
  static Dbg warn() { return Dbg(Dbg::Warn, "ReqHdlr"); }
  static Dbg trace() { return Dbg(Dbg::Trace, "ReqHdlr"); }

  Controller_if *_ctrl;
  l4_uint16_t _addr;
};

class I2c_virtio_device
: public L4virtio::Svr::Virtio_i2c<I2c_virtio_request_handler>
{
public:
  I2c_virtio_device(
    I2c_virtio_request_handler *req_hdlr,
    L4Re::Util::Object_registry *registry)
  : Virtio_i2c(req_hdlr, registry)
  {}
};

class I2c_factory : public L4::Epiface_t<I2c_factory, L4::Factory>
{
  enum Device_type
  {
    Type_virtio = 0,
    Type_rpc = 1,
  };

public:
  I2c_factory(L4Re::Util::Object_registry *registry, Controller_if *ctrl)
  : _ctrl(ctrl), _registry(registry)
  {}

  /**
   * Establish a connection to an i2c device.
   *
   * \param[out] res   Capability to access device.
   * \param      type  Object protocol: [0, 1]
   * \param      args  Arguments for device creation in order: i2c device address
   */
  long op_create(L4::Factory::Rights, L4::Ipc::Cap<void> &res,
                 l4_umword_t type, L4::Ipc::Varg_list<> &&args);

private:
  static Dbg warn() { return Dbg(Dbg::Warn, "Fab"); }
  static Dbg trace() { return Dbg(Dbg::Trace, "Fab"); }

  bool device_address_free(unsigned dev_addr) const;
  bool device_address_exists(unsigned dev_addr);

  Controller_if *_ctrl;
  L4Re::Util::Object_registry *_registry;
  std::vector<std::tuple<std::unique_ptr<I2c_virtio_device>,
                         std::unique_ptr<I2c_virtio_request_handler>>>
    _devices_virtio;
  std::vector<std::unique_ptr<I2c_device>> _devices_rpc;
};

bool
I2c_factory::device_address_free(unsigned dev_addr) const
{
  l4_uint16_t addr = dev_addr & 0xffff;

  for (auto const &[dev, hdlr] : _devices_virtio)
      if (hdlr->match(addr))
        return false;

  for (auto const &dev : _devices_rpc)
    if (dev->match(addr))
      return false;

  return true;
}

bool
I2c_factory::device_address_exists(unsigned dev_addr)
{
  l4_uint8_t buf[1] = {0x0};
  long ret = _ctrl->write(dev_addr, buf, 1U);
  return ret != -L4_ENODEV || ret >= 0;
}

long
I2c_factory::op_create(L4::Factory::Rights, L4::Ipc::Cap<void> &res,
                       l4_umword_t type, L4::Ipc::Varg_list<> &&args)
{
  warn().printf("Received create request for type %lu\n", type);
  if (type > 1)
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

  if (!device_address_free(dev_addr))
    return -L4_EEXIST;

  if (!device_address_exists(dev_addr))
    return -L4_ENODEV;

  L4::Cap<void> device_ep;

  switch (type)
    {
    case Type_virtio:
      {
        std::unique_ptr<I2c_virtio_request_handler> rqh =
          std::make_unique<I2c_virtio_request_handler>(_ctrl, dev_addr);
        if (!rqh)
          return -L4_EINVAL;

        std::unique_ptr<I2c_virtio_device> i2c_dev =
          std::make_unique<I2c_virtio_device>(rqh.get(), _registry);
        if (!i2c_dev)
          return -L4_EINVAL;

        device_ep = _registry->register_obj(i2c_dev.get());
        if (!device_ep)
          return -L4_EINVAL;

        trace().printf("Created device for addr 0x%lx: %p\n", dev_addr,
                       i2c_dev.get());

        _devices_virtio.push_back(
          std::make_tuple(std::move(i2c_dev), std::move(rqh)));
        break;
      }
    case Type_rpc:
      {
        std::unique_ptr<I2c_device> i2c_dev =
          std::make_unique<I2c_device>(dev_addr, _ctrl);

        if (!i2c_dev)
          return -L4_EINVAL;

        device_ep = _registry->register_obj(i2c_dev.get());
        if (!device_ep)
          return -L4_EINVAL;

        trace().printf("Created device for addr 0x%lx: %p\n", dev_addr,
                       i2c_dev.get());

        _devices_rpc.push_back(std::move(i2c_dev));
        break;
      }

    default: return -L4_ENODEV;
    }

  res = L4::Ipc::make_cap_rw(device_ep);

  return L4_EOK;
}

static std::tuple<L4::Cap<void>, std::unique_ptr<I2c_factory>>
init_factory(L4Re::Util::Object_registry *registry, char const *cap_name,
             Controller_if *ctrl)
{
  Dbg warn(Dbg::Warn, "Fab");
  Dbg trace(Dbg::Trace, "Fab");

  std::unique_ptr<I2c_factory> factory =
    std::make_unique<I2c_factory>(registry, ctrl);

  L4::Cap<void> cap = registry->register_obj(factory.get(), cap_name);

  if (!cap)
    {
      warn.printf("Capability %s for factory interface not in capability table. No i2c devices can be connected.\n",
                   cap_name);
      return std::make_tuple(cap, nullptr);
    }
  else
    trace.printf("factory cap 0x%lx\n", factory->obj_cap().cap());


  return std::make_tuple(cap, std::move(factory));
}

void start_server(Controller_if *ctrl)
{
  using Reg_Server =
    L4Re::Util::Registry_server<L4Re::Util::Br_manager_hooks>;

  Dbg warn(Dbg::Warn, "Srv");
  Dbg trace(Dbg::Trace, "Srv");

  // use myself as server thread.
  auto server = std::make_shared<Reg_Server>(Pthread::L4::cap(pthread_self()),
                                             L4Re::Env::env()->factory());

  ctrl->setup(server->registry());

  auto [factory_cap, factory] =
    init_factory(server->registry(), "dev_factory", ctrl);

  if (factory != nullptr)
    {
      trace.printf("factory initialized. start server loop\n");
      // start server loop to allow for factory connections.
      server->loop();

      warn.printf("exited server loop. Cleaning up ...\n");
      server->registry()->unregister_obj(factory.get());
      factory_cap.invalidate();
    }
  else
    warn.printf("Factory registration failed.\n");;
}
