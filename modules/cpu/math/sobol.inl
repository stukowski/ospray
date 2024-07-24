// Copyright 2009 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// Copyright (c) 2012 Leonhard Gruenschloss (leonhard@gruenschloss.org)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// These matrices are based on the following publication:
//
// S. Joe and F. Y. Kuo: "Constructing Sobol sequences with better
// two-dimensional projections", SIAM J. Sci. Comput. 30, 2635-2654 (2008).
//
// The tabulated direction numbers are available here:
// http://web.maths.unsw.edu.au/~fkuo/sobol/new-joe-kuo-6.21201

// bit-reversed and interleaved
OSPRAY_GLOBAL const unsigned int Sobol_revMatrices[4 * 32] = {
    0x1u,
    0x1u,
    0x1u,
    0x1u,

    0x3u,
    0x3u,
    0x3u,
    0x2u,

    0x5u,
    0x6u,
    0x4u,
    0x4u,

    0xfu,
    0x9u,
    0xau,
    0xdu,

    0x11u,
    0x17u,
    0x1fu,
    0x1fu,

    0x33u,
    0x3au,
    0x2eu,
    0x3bu,

    0x55u,
    0x71u,
    0x45u,
    0x5eu,

    0xffu,
    0xa3u,
    0xc9u,
    0xb9u,

    0x101u,
    0x116u,
    0x11bu,
    0x15au,

    0x303u,
    0x339u,
    0x2a4u,
    0x3f4u,

    0x505u,
    0x677u,
    0x79au,
    0x685u,

    0xf0fu,
    0x9aau,
    0xb67u,
    0xd0fu,

    0x1111u,
    0x1601u,
    0x101eu,
    0x115bu,

    0x3333u,
    0x3903u,
    0x302du,
    0x23f6u,

    0x5555u,
    0x7706u,
    0x4041u,
    0x4681u,

    0xffffu,
    0xaa09u,
    0xa0c3u,
    0xdd02u,

    0x10001u,
    0x10117u,
    0x1f104u,
    0x1e144u,

    0x30003u,
    0x3033au,
    0x2e28au,
    0x393cdu,

    0x50005u,
    0x60671u,
    0x457dfu,
    0x5a6dfu,

    0xf000fu,
    0x909a3u,
    0xc9baeu,
    0xb4dbbu,

    0x110011u,
    0x171616u,
    0x11a105u,
    0x14401eu,

    0x330033u,
    0x3a3939u,
    0x2a7289u,
    0x3cd039u,

    0x550055u,
    0x717777u,
    0x79e7dbu,
    0x6df05au,

    0xff00ffu,
    0xa3aaaau,
    0xb6dba4u,
    0xdbb0b4u,

    0x1010101u,
    0x1170001u,
    0x100011au,
    0x101e145u,

    0x3030303u,
    0x33a0003u,
    0x30002a7u,
    0x20393cfu,

    0x5050505u,
    0x6710006u,
    0x400079eu,
    0x405a6dbu,

    0xf0f0f0fu,
    0x9a30009u,
    0xa000b6du,
    0xd0b4db6u,

    0x11111111u,
    0x16160017u,
    0x1f001001u,
    0x1f144001u,

    0x33333333u,
    0x3939003au,
    0x2e003003u,
    0x3b3cd002u,

    0x55555555u,
    0x77770071u,
    0x45004004u,
    0x5e6df004u,

    0xffffffffu,
    0xaaaa00a3u,
    0xc900a00au,
    0xb9dbb00du,
};

OSPRAY_GLOBAL const unsigned int Sobol_matrices[5 * 52] = {
    0x80000000U,
    0x40000000U,
    0x20000000U,
    0x10000000U,
    0x8000000U,
    0x4000000U,
    0x2000000U,
    0x1000000U,
    0x800000U,
    0x400000U,
    0x200000U,
    0x100000U,
    0x80000U,
    0x40000U,
    0x20000U,
    0x10000U,
    0x8000U,
    0x4000U,
    0x2000U,
    0x1000U,
    0x800U,
    0x400U,
    0x200U,
    0x100U,
    0x80U,
    0x40U,
    0x20U,
    0x10U,
    0x8U,
    0x4U,
    0x2U,
    0x1U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x0U,
    0x80000000U,
    0xc0000000U,
    0xa0000000U,
    0xf0000000U,
    0x88000000U,
    0xcc000000U,
    0xaa000000U,
    0xff000000U,
    0x80800000U,
    0xc0c00000U,
    0xa0a00000U,
    0xf0f00000U,
    0x88880000U,
    0xcccc0000U,
    0xaaaa0000U,
    0xffff0000U,
    0x80008000U,
    0xc000c000U,
    0xa000a000U,
    0xf000f000U,
    0x88008800U,
    0xcc00cc00U,
    0xaa00aa00U,
    0xff00ff00U,
    0x80808080U,
    0xc0c0c0c0U,
    0xa0a0a0a0U,
    0xf0f0f0f0U,
    0x88888888U,
    0xccccccccU,
    0xaaaaaaaaU,
    0xffffffffU,
    0x80000000U,
    0xc0000000U,
    0xa0000000U,
    0xf0000000U,
    0x88000000U,
    0xcc000000U,
    0xaa000000U,
    0xff000000U,
    0x80800000U,
    0xc0c00000U,
    0xa0a00000U,
    0xf0f00000U,
    0x88880000U,
    0xcccc0000U,
    0xaaaa0000U,
    0xffff0000U,
    0x80008000U,
    0xc000c000U,
    0xa000a000U,
    0xf000f000U,
    0x80000000U,
    0xc0000000U,
    0x60000000U,
    0x90000000U,
    0xe8000000U,
    0x5c000000U,
    0x8e000000U,
    0xc5000000U,
    0x68800000U,
    0x9cc00000U,
    0xee600000U,
    0x55900000U,
    0x80680000U,
    0xc09c0000U,
    0x60ee0000U,
    0x90550000U,
    0xe8808000U,
    0x5cc0c000U,
    0x8e606000U,
    0xc5909000U,
    0x6868e800U,
    0x9c9c5c00U,
    0xeeee8e00U,
    0x5555c500U,
    0x8000e880U,
    0xc0005cc0U,
    0x60008e60U,
    0x9000c590U,
    0xe8006868U,
    0x5c009c9cU,
    0x8e00eeeeU,
    0xc5005555U,
    0x68808000U,
    0x9cc0c000U,
    0xee606000U,
    0x55909000U,
    0x8068e800U,
    0xc09c5c00U,
    0x60ee8e00U,
    0x9055c500U,
    0xe880e880U,
    0x5cc05cc0U,
    0x8e608e60U,
    0xc590c590U,
    0x68686868U,
    0x9c9c9c9cU,
    0xeeeeeeeeU,
    0x55555555U,
    0x80000000U,
    0xc0000000U,
    0x60000000U,
    0x90000000U,
    0x80000000U,
    0xc0000000U,
    0x20000000U,
    0x50000000U,
    0xf8000000U,
    0x74000000U,
    0xa2000000U,
    0x93000000U,
    0xd8800000U,
    0x25400000U,
    0x59e00000U,
    0xe6d00000U,
    0x78080000U,
    0xb40c0000U,
    0x82020000U,
    0xc3050000U,
    0x208f8000U,
    0x51474000U,
    0xfbea2000U,
    0x75d93000U,
    0xa0858800U,
    0x914e5400U,
    0xdbe79e00U,
    0x25db6d00U,
    0x58800080U,
    0xe54000c0U,
    0x79e00020U,
    0xb6d00050U,
    0x800800f8U,
    0xc00c0074U,
    0x200200a2U,
    0x50050093U,
    0xf80f80d8U,
    0x74074025U,
    0xa20a2059U,
    0x930930e6U,
    0xd88d8878U,
    0x254254b4U,
    0x59e59e82U,
    0xe6de6dc3U,
    0x780f80a0U,
    0xb4074091U,
    0x820a20dbU,
    0xc3093025U,
    0x208d8858U,
    0x514254e5U,
    0xfbe59e79U,
    0x75de6db6U,
    0xa08f8000U,
    0x91474000U,
    0xdbea2000U,
    0x25d93000U,
    0x80000000U,
    0x40000000U,
    0x20000000U,
    0xb0000000U,
    0xf8000000U,
    0xdc000000U,
    0x7a000000U,
    0x9d000000U,
    0x5a800000U,
    0x2fc00000U,
    0xa1600000U,
    0xf0b00000U,
    0xda880000U,
    0x6fc40000U,
    0x81620000U,
    0x40bb0000U,
    0x22878000U,
    0xb3c9c000U,
    0xfb65a000U,
    0xddb2d000U,
    0x78022800U,
    0x9c0b3c00U,
    0x5a0fb600U,
    0x2d0ddb00U,
    0xa2878080U,
    0xf3c9c040U,
    0xdb65a020U,
    0x6db2d0b0U,
    0x800228f8U,
    0x400b3cdcU,
    0x200fb67aU,
    0xb00ddb9dU,
    0xf80780daU,
    0xdc09c06fU,
    0x7a05a081U,
    0x9d02d040U,
    0x5a8a2822U,
    0x2fcf3cb3U,
    0xa16db6fbU,
    0xf0b6dbddU,
    0xda8000f8U,
    0x6fc000dcU,
    0x8160007aU,
    0x40b0009dU,
    0x2288005aU,
    0xb3c4002fU,
    0xfb6200a1U,
    0xddbb00f0U,
    0x780780daU,
    0x9c09c06fU,
    0x5a05a081U,
    0x2d02d040U,
};