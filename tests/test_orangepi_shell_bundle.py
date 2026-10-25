"""Validate executable shell bytes in the delivered ZIP, without device actions."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('package_orangepi_client', ROOT/'tools/package_orangepi_client.py')
package = importlib.util.module_from_spec(spec)
spec.loader.exec_module(package)


class ShellBundleTests(unittest.TestCase):
    def test_windows_checkout_bytes_and_bom_are_portable(self):
        windows_script = b'\xef\xbb\xbf#!/bin/sh\r\nset -eu\r\nprintf "ready\\n"\r\n'
        self.assertEqual(package.portable_entry('tools/start.sh', windows_script),
                         b'#!/bin/sh\nset -eu\nprintf "ready\\n"\n')
        binary = b'\x00\r\n\xff'
        self.assertEqual(package.portable_entry('model.onnx', binary), binary)

    def test_real_code_zip_scripts_execute_preamble_and_preserve_manifest(self):
        shell = Path('C:/Program Files/Git/usr/bin/sh.exe')
        shell = str(shell) if shell.is_file() else shutil.which('sh')
        with tempfile.TemporaryDirectory(prefix='acg720-shell-', dir=ROOT/'tmp') as folder:
            archive_path = Path(folder)/'code.zip'
            subprocess.run([sys.executable,str(ROOT/'tools/package_orangepi_client.py'),
                            '--code-only','--output',str(archive_path)],
                           check=True,capture_output=True,text=True)
            with zipfile.ZipFile(archive_path) as archive:
                manifest = json.loads(archive.read('acg720_vision/manifest.json'))
                self.assertEqual(manifest['bundle_revision'],2)
                self.assertEqual(manifest['shell_line_endings'],'LF')
                self.assertIsNone(archive.testzip())
                for name, expected in manifest['sha256'].items():
                    data = archive.read('acg720_vision/'+name)
                    self.assertEqual(hashlib.sha256(data).hexdigest(),expected,name)
                    if name.endswith('.sh'):
                        self.assertNotIn(b'\r',data,name)
                        self.assertTrue(data.startswith(b'#!/bin/sh\n'),name)
                        extracted = Path(folder)/Path(name).name
                        extracted.write_bytes(data)
                        if shell:
                            result = subprocess.run([shell,'-n',str(extracted)],capture_output=True,text=True)
                            self.assertEqual(result.returncode,0,result.stderr)
                names = archive.namelist()
                self.assertNotIn('acg720_vision/config/gimbal.json',names)
                self.assertNotIn('acg720_vision/config/visual_task.json',names)
                self.assertFalse(any(n.startswith('acg720_vision/models/') for n in names))
                # Execute only the actual launcher preamble through set -eu.
                # Stop before systemctl, processes, devices or network are touched.
                launcher=archive.read('acg720_vision/tools/start_fpga_follow.sh').decode('utf-8')
                prefix=launcher[:launcher.index('set -eu\n')+len('set -eu\n')]
                if shell:
                    result=subprocess.run([shell,'-c',prefix+"printf '%s\\n' LF_OK\n"],capture_output=True,text=True)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertEqual(result.stdout.strip(),'LF_OK')


if __name__ == '__main__':
    unittest.main()
