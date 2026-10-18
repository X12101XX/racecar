"""Filesystem and dependency isolation contracts for the native installer."""
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest import mock
from tools.install_bundle import extract_payload
from tools.native_runtime import verify_payload
from tools import native_runtime as runtime


def archive(files, manifest=None, extras=()):
    if manifest is None: manifest={n:hashlib.sha256(b).hexdigest() for n,b in files.items()}
    data=dict(files);data['MANIFEST.json']=json.dumps(manifest).encode()
    output=io.BytesIO()
    with tarfile.open(fileobj=output,mode='w:gz') as tar:
        for name,blob in data.items():
            info=tarfile.TarInfo('crossfire-car-v2/'+name);info.size=len(blob);info.mode=0o644
            tar.addfile(info,io.BytesIO(blob))
        for info in extras:tar.addfile(info)
    return output.getvalue()


class NativeInstallTests(unittest.TestCase):
    def test_portable_bundle_defaults_to_extract_without_running_payload(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            sentinel = root / 'unexpected preparation'
            payload = archive({'tools/build_car.sh':
                ('#!/bin/sh\ntouch "' + str(sentinel) + '"\n').encode()})
            installer = Path(__file__).resolve().parents[1] / 'tools/install_bundle.py'
            code = installer.read_text().replace('PAYLOAD_HASH', hashlib.sha256(payload).hexdigest())
            bundle = root / 'portable.run'
            bundle.write_bytes(("#!/bin/bash\nexec python3 -B - \"$0\" \"$@\" <<'PY'\n" + code +
                                '\nPY\n__CROSSFIRE_PAYLOAD__\n').encode() + payload)
            target = root / '自选路径 with spaces' / 'car'
            result = subprocess.run(['bash', str(bundle), '--target', str(target)],
                                    cwd='/', capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('绿色解压完成', result.stdout)
            self.assertFalse(sentinel.exists())
            self.assertFalse((target / '.runtime').exists())
            # Repeating an install must preserve even modified user data in that prefix.
            (target / 'keep.txt').write_text('keep')
            result = subprocess.run(['bash', str(bundle), '--target', str(target)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertEqual((target / 'keep.txt').read_text(), 'keep')
            prepared = root / 'explicit preparation'
            result = subprocess.run(['bash', str(bundle), '--target', str(prepared), '--prepare'],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(sentinel.exists())

    def test_new_install_does_not_change_existing_programs_or_shell(self):
        with tempfile.TemporaryDirectory() as tmp:
            home=Path(tmp);old=home/'racecar';old.mkdir();(old/'driver.py').write_text('existing driver')
            (home/'.bashrc').write_text('existing shell settings')
            files={'tools/example.py':b'print(1)','config/field.json':b'{}'}
            extract_payload(archive(files),home/'new_release')
            self.assertEqual((old/'driver.py').read_text(),'existing driver')
            self.assertEqual((home/'.bashrc').read_text(),'existing shell settings')
            self.assertEqual((home/'new_release/tools/example.py').read_bytes(),files['tools/example.py'])
            self.assertFalse((home/'new_release/.runtime').exists()) # Extraction never starts/prepares itself.

    def test_existing_directory_and_symlink_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'old';target.mkdir();(target/'important').write_text('keep')
            for dest in (target,Path(tmp)/'link'):
                if dest!=target:dest.symlink_to(target,target_is_directory=True)
                with self.assertRaisesRegex(ValueError,'拒绝覆盖'):extract_payload(archive({}),dest)
            self.assertEqual((target/'important').read_text(),'keep')

    def test_existing_file_and_dangling_symlink_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            file = root / 'vendor'; file.write_text('keep')
            link = root / 'dangling'; link.symlink_to(root / 'nonexistent')
            for target in (file, link):
                with self.assertRaisesRegex(ValueError, '拒绝覆盖'):
                    extract_payload(archive({}), target)
            self.assertEqual(file.read_text(), 'keep')
            self.assertTrue(link.is_symlink())

    def test_runtime_links_are_rejected_before_dependency_probe(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); external = root / 'vendor'; external.mkdir()
            (external / 'important').write_text('keep')
            package = root / 'package'; package.mkdir()
            for link_name in ('.runtime', '.runtime/car'):
                link = package / link_name
                link.parent.mkdir(exist_ok=True)
                link.symlink_to(external, target_is_directory=True)
                with mock.patch.object(runtime, 'ROOT', package), \
                     mock.patch.object(runtime, 'RUNTIME', package / '.runtime/car'), \
                     mock.patch.object(runtime, 'probe') as probe:
                    with self.assertRaisesRegex(ValueError, '拒绝写入'):
                        runtime.prepare(root / 'missing', '/usr/bin/python3')
                    probe.assert_not_called()
                link.unlink()
            self.assertEqual((external / 'important').read_text(), 'keep')
            self.assertEqual(len(list(external.iterdir())), 1)

    def test_unowned_runtime_and_hardlinks_are_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); private = root / '.runtime/car'; private.mkdir(parents=True)
            important = private / 'runtime.json'; important.write_text('unrelated existing file')
            with mock.patch.object(runtime, 'ROOT', root), mock.patch.object(runtime, 'RUNTIME', private):
                with self.assertRaisesRegex(ValueError, '拒绝覆盖'):
                    runtime.check_runtime_directory()
                self.assertEqual(important.read_text(), 'unrelated existing file')
                (private / runtime.RUNTIME_OWNER).write_text(json.dumps(runtime.OWNER_RECORD))
                external = root / 'external'; external.write_text('keep')
                os.link(external, private / 'cache-file')
                with self.assertRaisesRegex(ValueError, '硬链接'):
                    runtime.check_runtime_directory()
                self.assertEqual(external.read_text(), 'keep')

    def test_runtime_state_is_relocatable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'before'; private = root / '.runtime/car'; private.mkdir(parents=True)
            source = root / 'racelib/native/hybrid_astar_core.cpp'; source.parent.mkdir(parents=True)
            source.write_text('source')
            (private / runtime.RUNTIME_OWNER).write_text(json.dumps(runtime.OWNER_RECORD))
            import platform
            data = dict(architecture=platform.machine(), native_available=False,
                        native_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
            (private / 'runtime.json').write_text(json.dumps(data))
            moved = root.with_name('搬动后 with spaces'); root.rename(moved)
            with mock.patch.object(runtime, 'ROOT', moved), \
                 mock.patch.object(runtime, 'RUNTIME', moved / '.runtime/car'):
                self.assertEqual(runtime.state(), data)

    def test_corrupt_manifest_rejected_before_creating_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'new'
            with self.assertRaisesRegex(ValueError,'校验失败'):
                extract_payload(archive({'a.py':b'changed'},{'a.py':'0'*64}),target)
            self.assertFalse(target.exists())

    def test_traversal_and_symlink_members_rejected(self):
        for name,kind in [('crossfire-car-v2/../../victim',tarfile.REGTYPE),
                          ('crossfire-car-v2/link',tarfile.SYMTYPE)]:
            info=tarfile.TarInfo(name);info.type=kind;info.linkname='/etc'
            with tempfile.TemporaryDirectory() as tmp:
                target=Path(tmp)/'new'
                with self.assertRaises(ValueError):extract_payload(archive({},extras=[info]),target)
                self.assertFalse(target.exists())

    def test_extra_unlisted_payload_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'清单不一致'):
                extract_payload(archive({'unlisted.py':b'print(1)'},{}),Path(tmp)/'new')

    def test_code_integrity_and_editable_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'new';extract_payload(archive({'entry.py':b'original','config/plan.json':b'{}'}),root)
            (root/'config/plan.json').write_text('{"user":true}')
            verify_payload(root)
            (root/'entry.py').write_text('tampered')
            with self.assertRaisesRegex(ValueError,'校验失败'):verify_payload(root)

    def test_missing_ros_creates_no_private_or_global_runtime(self):
        from tools import native_runtime as runtime
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);private=root/'release';private.mkdir()
            (root/'.bashrc').write_text('keep')
            with mock.patch.object(runtime,'ROOT',private),mock.patch.object(runtime,'RUNTIME',private/'.runtime/car'):
                with self.assertRaisesRegex(ValueError,'未找到已有 ROS2'):
                    runtime.prepare(root/'missing_ros/setup.bash','/usr/bin/python3')
            self.assertFalse((private/'.runtime').exists())
            self.assertEqual((root/'.bashrc').read_text(),'keep')

    def test_global_environment_not_modified(self):
        import os
        from tools.native_runtime import environment
        with mock.patch.dict(os.environ,{'BASH_ENV':'unexpected.sh','PYTHONHOME':'external','UNRELATED':'keep'}):
            result=environment()
            self.assertNotIn('BASH_ENV',result);self.assertNotIn('PYTHONHOME',result)
            self.assertEqual(os.environ['BASH_ENV'],'unexpected.sh')
            self.assertEqual(result['UNRELATED'],'keep')


if __name__=='__main__':unittest.main()
