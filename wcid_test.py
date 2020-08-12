#!/usr/bin/env python3

from __future__ import print_function

import usb
import argparse
import sys
import usb.backend.libusb1

from usb.control import get_descriptor

def print_feature_descriptor(feat_desc):
    if feat_desc:
        print("Feature descriptor received")
        if len(feat_desc) == feat_desc[0]:
            print("Length: %d bytes" % feat_desc[0])
            if len(feat_desc) >= 6:
                print("Version: %d.%d" % (feat_desc[5], feat_desc[4]))
            if len(feat_desc) >= 26:
                print("Driver: " + ''.join([chr(x) for x in feat_desc[18:25]]))
        else:
            print("Length mismatch in feature descriptor")
            print(' '.join(["%02x" % x for x in feat_desc]))
    print()

def find_descriptors(idVendor, idProduct):
    dev = usb.core.find(idVendor=idVendor, idProduct=idProduct)
    if not dev:
        print("Could not find device %04x:%04x" % (idVendor, idProduct))
        return
    
    print("Found device:")
    print(dev._get_full_descriptor_str())
    print()
    print("USB %d.%d device found" % ((dev.bcdUSB & 0x0F00) >> 8, (dev.bcdUSB & 0x00F0) >> 4))
    if dev.bcdUSB < 0x0200:
        print("WCID descriptors are only valid for USB 2.0+")
        return

    DESC_TYPE_CONFIG = 0x02
    DESC_TYPE_STRING = 0x03
    MS_OS_STRING_IDX = 0xEE

    langids = dev.langids
    if not len(langids):
        print("The device has no language IDs")
        return

    buf = []
    for langid in langids:
        buf = get_descriptor(dev, 255, DESC_TYPE_STRING, MS_OS_STRING_IDX,
                             langid)
        if buf:
            break
    else:
        print("No Microsoft OS String Descriptor found")
        return
    
    ms_os_string = buf[2:buf[0]].tobytes().decode('utf-16-le')
    ms_vendor_req = buf[-2]
    if ms_os_string.startswith(u"MSFT100"):
        print("Found '%s' signature" % ms_os_string)
        print("Feature descriptor request 0x%02x" % ms_vendor_req)
        print()
    else:
        print("'MSFT100' signature not found")
        return

    try:
        feat_desc = dev.ctrl_transfer(0xC0, ms_vendor_req, 0, 0x0004, 16)
    except usb.core.USBError as e:
        print("Request failed with error: %s" % e)
        return
    if not feat_desc:
         print("No feature descriptor found")
         return
    feat_desc = dev.ctrl_transfer(0xC0, ms_vendor_req, 0, 0x0004, feat_desc[0])
    print_feature_descriptor(feat_desc)

    ext_feat_desc = dev.ctrl_transfer(0xC1, ms_vendor_req, 0, 0x0005, 255)
    if ext_feat_desc:
        print(' '.join(["%02x" % x for x in ext_feat_desc]))
    else:
        print("No extended feature descriptor found")

def vid_pid(x):
    return int(x, 16)

def main():


    #parser = argparse.ArgumentParser(description="A tool for checking a USB device's Windows compatibility ID")
    #parser.add_argument('-v', dest='vendorid', metavar='<VendorID>',
    #                    type=vid_pid, help="Vendor ID of device",
    #                    required=True)
    #parser.add_argument('-p', dest='productid', metavar='<ProductID>',
    #                    type=vid_pid, help="Product ID of device",
    #                    required=True)
    #try:
    #    args = parser.parse_args()
    #except:
    #    return "Invalid Args"
    
    usbVendorId = 0x04D8;
    usbProductId = 0xEC5A;

    find_descriptors(usbVendorId, usbProductId)

if __name__ == "__main__":
    main()