import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class GatewayFreshnessTests(unittest.TestCase):
    def test_boot_timeout_recovery_and_timer_wrap(self):
        compiler = shutil.which('g++')
        if not compiler and Path('C:/msys64/mingw64/bin/g++.exe').is_file():
            compiler = 'C:/msys64/mingw64/bin/g++.exe'
        if not compiler:
            self.skipTest('Requires g++')
        source = Path(__file__).parent / 'native/link_freshness_test.cpp'
        environment = dict(os.environ)
        environment['PATH'] = str(Path(compiler).parent) + os.pathsep + environment.get('PATH', '')
        with tempfile.TemporaryDirectory() as temp:
            binary = Path(temp) / 'link_freshness_test.exe'
            result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                                     '-static', str(source), '-o', str(binary)],
                                    capture_output=True, text=True, env=environment)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
