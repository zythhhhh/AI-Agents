"""Enumerate USB controllers without opening or moving them."""
import hid

def main():
    devices = hid.enumerate(0x0483, 0x5750)
    print("Matching USB interfaces:", len(devices))
    for index, device in enumerate(devices):
        print(index, device.get("product_string"), device.get("serial_number"), device.get("path"))
    print("Enumeration order is not a persistent arm identity.")

if __name__ == "__main__":
    main()
