"""Build private same-source query modules; invoked only in the image builder."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import sysconfig
import tempfile


def build(source_dir, output_dir):
    import Cython
    from Cython.Build import cythonize
    from setuptools import Extension, setup

    if sys.version_info[:2] != (3, 8) or Cython.__version__ != '3.2.9':
        raise RuntimeError('query extensions require Noetic CPython3.8 and Cython3.2.9')
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError('query extension output directory must be empty')
    modules = ('qn_dynamics', 'qn_python_backend')
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    manifest = dict(schema_version=1, cache_tag=sys.implementation.cache_tag,
                    implementation=sys.implementation.name, extension_suffix=suffix,
                    python=sys.version, cython=Cython.__version__, modules={})
    with tempfile.TemporaryDirectory(prefix='qn-query-build-') as temporary:
        package = Path(temporary) / 'qn_aav_simulator'
        package.mkdir()
        (package / '__init__.py').write_text('')
        for name in modules:
            content = (source_dir / (name + '.py')).read_bytes()
            (package / (name + '.py')).write_bytes(content)
            manifest['modules'][name] = dict(source_sha256=hashlib.sha256(content).hexdigest())
        previous_directory = Path.cwd()
        try:
            os.chdir(temporary)
            extensions = []
            for name in modules:
                typed = name == 'qn_python_backend'
                extension = Extension('qn_aav_simulator.' + name,
                    ['qn_aav_simulator/' + name + '.py'],
                    extra_compile_args=['-fno-fast-math', '-ffp-contract=off'])
                extensions += cythonize([extension], compiler_directives={
                    'language_level': 3, 'infer_types': typed, 'annotation_typing': typed},
                    force=True)
            setup(name='qn-private-query', ext_modules=extensions,
                  script_args=['build_ext', '--inplace'])
        finally:
            os.chdir(previous_directory)
        output_dir.mkdir(parents=True, exist_ok=True)
        for name in modules:
            filename = name + suffix
            binary = package / filename
            manifest['modules'][name].update(filename=filename,
                binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
            shutil.copy2(binary, output_dir / filename)
        (output_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    arguments = parser.parse_args()
    build(arguments.source_dir.resolve(), arguments.output_dir.resolve())
