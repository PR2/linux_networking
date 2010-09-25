#! /usr/bin/env python

import re

def is_packed(pk):
    return type(pk) == str and len(pk) == 6

def packed_to_str(pk):
    if is_packed(pk):
        return ":".join(("%02x"%ord(pk[i]) for i in range(6)))
    raise ValueError("A packed MAC address is a 6 character string.")

str_re_str = '^%s$'%(':'.join(6*[2*'[0-9A-Fa-f]']))
str_re = re.compile(str_re_str)

def is_str(str):
    return str_re.search(str) is not None

def str_to_packed(str):
    if is_str(str):
        return "".join(chr(int(h,16)) for h in str.split(':'))
    raise ValueError("A MAC address string is 6 two digit hex numbers separated by colons.")

if __name__ == "__main__":
    import unittest
    import sys
        
    class BasicTest(unittest.TestCase):
        def test_bad_packed(self):
            self.assertRaises(ValueError, packed_to_str, "12345")
            self.assertRaises(ValueError, packed_to_str, "1234567")
            self.assertRaises(ValueError, packed_to_str, 12)
            self.assertRaises(ValueError, packed_to_str, [])

        def test_bad_str(self):
            self.assertRaises(ValueError, str_to_packed, "12:34:56:78:90")
            self.assertRaises(ValueError, str_to_packed, "12:34:56:78:90:ab:cd")
            self.assertRaises(ValueError, str_to_packed, "aoeu")
            self.assertRaises(ValueError, str_to_packed, "gg:12:34:56:78:90")

        def test_conv(self):
            self.assertEqual("blaise", str_to_packed("62:6c:61:69:73:65"))
            self.assertEqual("blaise", str_to_packed("62:6C:61:69:73:65"))
            self.assertEqual("blaise", str_to_packed(packed_to_str("blaise")))
            self.assertEqual("ct,cu ", str_to_packed(packed_to_str("ct,cu ")))

    if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
        import roslib; roslib.load_manifest('multi_interface_roam')
        import rostest
        rostest.unitrun('multi_interface_roam', 'addr_basic', BasicTest)
    else:
        unittest.main()
