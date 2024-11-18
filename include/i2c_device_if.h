/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Philipp Eppelt philipp.eppelt@kernkonzept.com
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */
#pragma once

#include <l4/sys/kobject>
#include <l4/sys/cxx/ipc_epiface>


struct I2c_device_ops
: public L4::Kobject_t<I2c_device_ops, L4::Kobject, L4::PROTO_ANY,
                       L4::Type_info::Demand_t<1>>
{
  L4_INLINE_RPC(long, read, (L4::Ipc::Array<l4_uint8_t> &));
  L4_INLINE_RPC(long, write, (L4::Ipc::Array<l4_uint8_t const>));

//  L4_INLINE_RPC(long, read_dma, (l4_addr_t, unsigned));
//  L4_INLINE_RPC(long, write_dma, (l4_addr_t, unsigned));

  using Rpcs = L4::Typeid::Rpcs<read_t, write_t>;
};

