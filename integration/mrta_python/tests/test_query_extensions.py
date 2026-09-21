"""A stale or unusable private build must leave the current source importable."""
import hashlib
import importlib
import json
import sys
import sysconfig
from types import ModuleType

import pytest

from mrta_python import query_worker


@pytest.mark.parametrize('fault', (
    'missing_build', 'wrong_abi', 'changed_dynamics', 'changed_backend',
    'missing_binary', 'changed_binary', 'unloadable_binary',
))
def test_private_extension_faults_fall_back_before_snapshot_import(tmp_path, monkeypatch, fault):
    source = tmp_path / 'source'
    private = tmp_path / 'private'
    source.mkdir(); private.mkdir()
    package = ModuleType('qn_aav_simulator')
    package.__path__ = [str(source)]
    monkeypatch.setitem(sys.modules, 'qn_aav_simulator', package)
    monkeypatch.setattr(query_worker, '_QUERY_EXTENSIONS', private)
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    manifest = dict(schema_version=1, cache_tag=sys.implementation.cache_tag,
        implementation=sys.implementation.name, extension_suffix=suffix, modules={})
    for name in query_worker._QN_MODULES:
        full_name = 'qn_aav_simulator.' + name
        # Register restoration even if the module was absent before this test.
        monkeypatch.setitem(sys.modules, full_name, None)
        del sys.modules[full_name]
        content = b'ORIGIN = "current Python source"\n'
        (source / (name + '.py')).write_bytes(content)
        filename = name + suffix
        binary = b'not a loadable extension'
        (private / filename).write_bytes(binary)
        manifest['modules'][name] = dict(filename=filename,
            source_sha256=hashlib.sha256(content).hexdigest(),
            binary_sha256=hashlib.sha256(binary).hexdigest())
    if fault == 'wrong_abi':
        manifest['cache_tag'] = 'cpython-wrong-version'
    if fault in ('changed_dynamics', 'changed_backend'):
        name = 'qn_dynamics' if fault == 'changed_dynamics' else 'qn_python_backend'
        (source / (name + '.py')).write_text('ORIGIN = "updated Python source"\n')
    if fault == 'missing_binary':
        (private / ('qn_python_backend' + suffix)).unlink()
    if fault == 'changed_binary':
        (private / ('qn_python_backend' + suffix)).write_bytes(b'stale binary replacement')
    if fault != 'missing_build':
        (private / 'manifest.json').write_text(json.dumps(manifest))

    assert query_worker._enable_query_extensions() is False
    assert package.__path__ == [str(source)]
    for name in query_worker._QN_MODULES:
        module = importlib.import_module('qn_aav_simulator.' + name)
        assert module.__file__ == str(source / (name + '.py'))
        assert module.ORIGIN in ('current Python source', 'updated Python source')
