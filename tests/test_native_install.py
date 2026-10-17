"""Filesystem and dependency isolation contracts for the native installer."""
import hashlib
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest import mock
from tools.install_bundle import extract_payload
from tools.native_runtime import verify_payload


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
