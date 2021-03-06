
#ifndef CS2102USB_H
#define CS2102USB_H
/****************************************************************************
#	 	Century Semiconductor CS2102    library                     #
# 		Copyright (C) 2004 2005 Michel Xhaard   mxhaard@magic.fr    #
#               Copyright (C) 2005 Alvaro Salmador naplam33 at msn.com      #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

static __u16 cs2102_start_data[][3] = {
    {0xa1, 0x01, 0x0008},
    {0xa1, 0x01, 0x0008},
    {0xa0, 0x01, 0x0000},
    {0xa0, 0x10, 0x0002},
    {0xa0, 0x00, 0x0010},
    {0xa0, 0x01, 0x0001},
    {0xa0, 0x20, 0x0080},
    {0xa0, 0x21, 0x0081},
    {0xa0, 0x30, 0x0083},
    {0xa0, 0x31, 0x0084},
    {0xa0, 0x32, 0x0085},
    {0xa0, 0x23, 0x0086},
    {0xa0, 0x24, 0x0087},
    {0xa0, 0x25, 0x0088},
    {0xa0, 0xb3, 0x008b},
    {0xa0, 0x03, 0x0008},	//00
    {0xa0, 0x03, 0x0012},
    {0xa0, 0x01, 0x0012},
    {0xa0, 0x02, 0x0003},
    {0xa0, 0x80, 0x0004},
    {0xa0, 0x01, 0x0005},
    {0xa0, 0xe0, 0x0006},
    {0xa0, 0x00, 0x0098},
    {0xa0, 0x00, 0x009a},
    {0xa0, 0x00, 0x011a},
    {0xa0, 0x00, 0x011c},
    {0xa0, 0x02, 0x0092},
    {0xa0, 0x08, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x03, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x11, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x12, 0x0092},
    {0xa0, 0x89, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x13, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x14, 0x0092},
    {0xa0, 0xe9, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x20, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x22, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x0b, 0x0092},
    {0xa0, 0x04, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x30, 0x0092},
    {0xa0, 0x30, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x31, 0x0092},
    {0xa0, 0x30, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x32, 0x0092},
    {0xa0, 0x30, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x37, 0x0101},
    {0xa0, 0x00, 0x0019},
    {0xa0, 0x05, 0x0012},
    {0xa0, 0x0d, 0x0100},
    {0xa0, 0x06, 0x0189},
    {0xa0, 0x03, 0x01c5},
    {0xa0, 0x13, 0x01cb},
    {0xa0, 0x10, 0x01ae},
    {0xa0, 0x08, 0x0250},
    {0xa0, 0x08, 0x0301},
    {0xa0, 0x68, 0x018d},
    {0xa0, 0x00, 0x01ad},
    {0xa1, 0x01, 0x0002},
    {0xa1, 0x01, 0x0008},
    {0xa0, 0x03, 0x0008},	//00
    {0xa0, 0x08, 0x01c6},
    {0xa1, 0x01, 0x01c8},
    {0xa1, 0x01, 0x01c9},
    {0xa1, 0x01, 0x01ca},
    {0xa0, 0x0f, 0x01cb},
    {0xa0, 0x24, 0x0120},
    {0xa0, 0x44, 0x0121},
    {0xa0, 0x64, 0x0122},
    {0xa0, 0x84, 0x0123},
    {0xa0, 0x9d, 0x0124},
    {0xa0, 0xb2, 0x0125},
    {0xa0, 0xc4, 0x0126},
    {0xa0, 0xd3, 0x0127},
    {0xa0, 0xe0, 0x0128},
    {0xa0, 0xeb, 0x0129},
    {0xa0, 0xf4, 0x012a},
    {0xa0, 0xfb, 0x012b},
    {0xa0, 0xff, 0x012c},
    {0xa0, 0xff, 0x012d},
    {0xa0, 0xff, 0x012e},
    {0xa0, 0xff, 0x012f},
    {0xa0, 0x18, 0x0130},
    {0xa0, 0x20, 0x0131},
    {0xa0, 0x20, 0x0132},
    {0xa0, 0x1c, 0x0133},
    {0xa0, 0x16, 0x0134},
    {0xa0, 0x13, 0x0135},
    {0xa0, 0x10, 0x0136},
    {0xa0, 0x0e, 0x0137},
    {0xa0, 0x0b, 0x0138},
    {0xa0, 0x09, 0x0139},
    {0xa0, 0x07, 0x013a},
    {0xa0, 0x06, 0x013b},
    {0xa0, 0x00, 0x013c},
    {0xa0, 0x00, 0x013d},
    {0xa0, 0x00, 0x013e},
    {0xa0, 0x01, 0x013f},
    {0xa0, 0x58, 0x010a},
    {0xa0, 0xf4, 0x010b},
    {0xa0, 0xf4, 0x010c},
    {0xa0, 0xf4, 0x010d},
    {0xa0, 0x58, 0x010e},
    {0xa0, 0xf4, 0x010f},
    {0xa0, 0xf4, 0x0110},
    {0xa0, 0xf4, 0x0111},
    {0xa0, 0x58, 0x0112},
    {0xa1, 0x01, 0x0180},
    {0xa0, 0x00, 0x0180},
    {0xa0, 0x00, 0x0019},
    {0xa0, 0x23, 0x0092},
    {0xa0, 0x01, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x24, 0x0092},
    {0xa0, 0x55, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x25, 0x0092},
    {0xa0, 0xcc, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x21, 0x0092},
    {0xa0, 0x3f, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x02, 0x0190},
    {0xa0, 0xab, 0x0191},
    {0xa0, 0x98, 0x0192},
    {0xa0, 0x00, 0x0195},
    {0xa0, 0x30, 0x0196},
    {0xa0, 0xd4, 0x0197},
    {0xa0, 0x10, 0x018c},
    {0xa0, 0x20, 0x018f},
    {0xa0, 0x10, 0x01a9},
    {0xa0, 0x24, 0x01aa},
    {0xa0, 0x39, 0x001d},
    {0xa0, 0x70, 0x001e},
    {0xa0, 0xb0, 0x001f},
    {0xa0, 0xff, 0x0020},
    {0xa0, 0x40, 0x0180},
    {0xa1, 0x01, 0x0180},
    {0xa0, 0x42, 0x0180},
    {0xa0, 0x40, 0x0116},
    {0xa0, 0x40, 0x0117},
    {0xa0, 0x40, 0x0118},
    {0, 0, 0}
};


static __u16 cs2102_scale_data[][3] = {
    {0xa1, 0x01, 0x0008},
    {0xa1, 0x01, 0x0008},
    {0xa0, 0x01, 0x0000},
    {0xa0, 0x00, 0x0002},
    {0xa0, 0x00, 0x0010},
    {0xa0, 0x01, 0x0001},
    {0xa0, 0x20, 0x0080},
    {0xa0, 0x21, 0x0081},
    {0xa0, 0x30, 0x0083},
    {0xa0, 0x31, 0x0084},
    {0xa0, 0x32, 0x0085},
    {0xa0, 0x23, 0x0086},
    {0xa0, 0x24, 0x0087},
    {0xa0, 0x25, 0x0088},
    {0xa0, 0xb3, 0x008b},
    {0xa0, 0x03, 0x0008},	//00
    {0xa0, 0x03, 0x0012},
    {0xa0, 0x01, 0x0012},
    {0xa0, 0x02, 0x0003},
    {0xa0, 0x80, 0x0004},
    {0xa0, 0x01, 0x0005},
    {0xa0, 0xe0, 0x0006},
    {0xa0, 0x00, 0x0098},
    {0xa0, 0x00, 0x009a},
    {0xa0, 0x00, 0x011a},
    {0xa0, 0x00, 0x011c},
    {0xa0, 0x02, 0x0092},
    {0xa0, 0x08, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x03, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x11, 0x0092},
    {0xa0, 0x01, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x12, 0x0092},
    {0xa0, 0x87, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x13, 0x0092},
    {0xa0, 0x01, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x14, 0x0092},
    {0xa0, 0xe7, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x20, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x22, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x0b, 0x0092},
    {0xa0, 0x04, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x30, 0x0092},
    {0xa0, 0x30, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x31, 0x0092},
    {0xa0, 0x30, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x32, 0x0092},
    {0xa0, 0x30, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x77, 0x0101},
    {0xa0, 0x00, 0x0019},
    {0xa0, 0x05, 0x0012},
    {0xa0, 0x0d, 0x0100},
    {0xa0, 0x06, 0x0189},
    {0xa0, 0x03, 0x01c5},
    {0xa0, 0x13, 0x01cb},
    {0xa0, 0x15, 0x01ae},
    {0xa0, 0x08, 0x0250},
    {0xa0, 0x08, 0x0301},
    {0xa0, 0x68, 0x018d},
    {0xa0, 0x00, 0x01ad},
    {0xa1, 0x01, 0x0002},
    {0xa1, 0x01, 0x0008},
    {0xa0, 0x03, 0x0008},	//00
    {0xa0, 0x08, 0x01c6},
    {0xa1, 0x01, 0x01c8},
    {0xa1, 0x01, 0x01c9},
    {0xa1, 0x01, 0x01ca},
    {0xa0, 0x0f, 0x01cb},
    {0xa0, 0x24, 0x0120},
    {0xa0, 0x44, 0x0121},
    {0xa0, 0x64, 0x0122},
    {0xa0, 0x84, 0x0123},
    {0xa0, 0x9d, 0x0124},
    {0xa0, 0xb2, 0x0125},
    {0xa0, 0xc4, 0x0126},
    {0xa0, 0xd3, 0x0127},
    {0xa0, 0xe0, 0x0128},
    {0xa0, 0xeb, 0x0129},
    {0xa0, 0xf4, 0x012a},
    {0xa0, 0xfb, 0x012b},
    {0xa0, 0xff, 0x012c},
    {0xa0, 0xff, 0x012d},
    {0xa0, 0xff, 0x012e},
    {0xa0, 0xff, 0x012f},
    {0xa0, 0x18, 0x0130},
    {0xa0, 0x20, 0x0131},
    {0xa0, 0x20, 0x0132},
    {0xa0, 0x1c, 0x0133},
    {0xa0, 0x16, 0x0134},
    {0xa0, 0x13, 0x0135},
    {0xa0, 0x10, 0x0136},
    {0xa0, 0x0e, 0x0137},
    {0xa0, 0x0b, 0x0138},
    {0xa0, 0x09, 0x0139},
    {0xa0, 0x07, 0x013a},
    {0xa0, 0x06, 0x013b},
    {0xa0, 0x00, 0x013c},
    {0xa0, 0x00, 0x013d},
    {0xa0, 0x00, 0x013e},
    {0xa0, 0x01, 0x013f},
    {0xa0, 0x58, 0x010a},
    {0xa0, 0xf4, 0x010b},
    {0xa0, 0xf4, 0x010c},
    {0xa0, 0xf4, 0x010d},
    {0xa0, 0x58, 0x010e},
    {0xa0, 0xf4, 0x010f},
    {0xa0, 0xf4, 0x0110},
    {0xa0, 0xf4, 0x0111},
    {0xa0, 0x58, 0x0112},
    {0xa1, 0x01, 0x0180},
    {0xa0, 0x00, 0x0180},
    {0xa0, 0x00, 0x0019},
    {0xa0, 0x23, 0x0092},
    {0xa0, 0x00, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x24, 0x0092},
    {0xa0, 0xaa, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x25, 0x0092},
    {0xa0, 0xe6, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x21, 0x0092},
    {0xa0, 0x3f, 0x0093},
    {0xa0, 0x00, 0x0094},
    {0xa0, 0x01, 0x0090},
    {0xa1, 0x01, 0x0091},
    {0xa0, 0x01, 0x0190},
    {0xa0, 0x55, 0x0191},
    {0xa0, 0xcc, 0x0192},
    {0xa0, 0x00, 0x0195},
    {0xa0, 0x18, 0x0196},
    {0xa0, 0x6a, 0x0197},
    {0xa0, 0x10, 0x018c},
    {0xa0, 0x20, 0x018f},
    {0xa0, 0x10, 0x01a9},
    {0xa0, 0x24, 0x01aa},
    {0xa0, 0x3f, 0x001d},
    {0xa0, 0xa5, 0x001e},
    {0xa0, 0xf0, 0x001f},
    {0xa0, 0xff, 0x0020},
    {0xa0, 0x40, 0x0180},
    {0xa1, 0x01, 0x0180},
    {0xa0, 0x42, 0x0180},
    {0xa0, 0x40, 0x0116},
    {0xa0, 0x40, 0x0117},
    {0xa0, 0x40, 0x0118},
    {0, 0, 0}
};

#endif				//CS2102USB_H
