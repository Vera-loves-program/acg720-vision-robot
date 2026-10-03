"""Check Ubuntu URL rewriting without accessing or modifying system APT files."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("fix_apt", Path(__file__).resolve().parents[1] / "tools" / "fix_orangepi_apt.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PortsSourceTests(unittest.TestCase):
    def test_preserves_vendor_comments_suites_and_components(self):
        before = ("# deb http://old/ubuntu-ports jammy main\n"
                  "deb http://repo.huaweicloud.com/ubuntu-ports jammy-updates main universe # keep\n"
                  "deb https://repo.orangepi.org/debian jammy main\n"
                  "deb http://archive.ubuntu.com/ubuntu jammy main\n")
        after = module.replace_ports_urls(before)
        self.assertIn("deb https://mirrors.aliyun.com/ubuntu-ports/ jammy-updates main universe # keep", after)
        self.assertIn("# deb http://old/ubuntu-ports jammy main", after)
        self.assertIn("https://repo.orangepi.org/debian", after)
        self.assertIn("http://archive.ubuntu.com/ubuntu", after)

    def test_deb822_and_signed_by_are_preserved(self):
        before = "Types: deb\nURIs: http://ports.ubuntu.com/ubuntu-ports/\nSuites: jammy\nSigned-By: /usr/share/keyrings/ubuntu.gpg\n"
        after = module.replace_ports_urls(before)
        self.assertIn("URIs: " + module.ALI, after)
        self.assertIn("Signed-By: /usr/share/keyrings/ubuntu.gpg", after)
        self.assertEqual(module.replace_ports_urls(after), after)

    def test_options_and_credentials(self):
        before = "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main\n"
        self.assertIn("[arch=arm64] " + module.ALI, module.replace_ports_urls(before))
        private = "deb https://user:secret@example.test/ubuntu-ports jammy main\n"
        self.assertEqual(module.replace_ports_urls(private), private)


if __name__ == "__main__":
    unittest.main()
