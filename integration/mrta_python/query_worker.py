"""Private bounded-query subprocess; stdin is only from the trusted planner.

Do not expose this pickle channel as a network or user-file interface.
"""
import contextlib
import pickle
import sys
import struct
import hashlib
import importlib
import importlib.machinery
import json
from pathlib import Path
import sysconfig


_QUERY_EXTENSIONS = Path('/workspace/query_extensions')
_QN_MODULES = ('qn_dynamics', 'qn_python_backend')


def _enable_query_extensions():
    """Select only matching private builds, before any snapshot is unpickled.

    The ordinary qn package and all other modules still come from the current
    source checkout. Never replace already-loaded classes or change a parent's
    import path: their identities are part of the trusted pickle protocol.
    """
    qualified = tuple('qn_aav_simulator.' + name for name in _QN_MODULES)
    if any(name in sys.modules for name in qualified):
        return False
    try:
        manifest = json.loads((_QUERY_EXTENSIONS / 'manifest.json').read_text())
        suffix = sysconfig.get_config_var('EXT_SUFFIX')
        if (manifest['schema_version'] != 1 or
                manifest['cache_tag'] != sys.implementation.cache_tag or
                manifest['extension_suffix'] != suffix or
                manifest['implementation'] != sys.implementation.name):
            return False
        package = importlib.import_module('qn_aav_simulator')
        if any(name in sys.modules for name in qualified):
            return False
        for name, full_name in zip(_QN_MODULES, qualified):
            # Resolve against the unmodified source package search path, not
            # against the extension directory we may enable below.
            source = importlib.machinery.PathFinder.find_spec(full_name, package.__path__)
            if source is None or not source.origin or not source.origin.endswith('.py'):
                return False
            entry = manifest['modules'][name]
            filename = name + suffix
            if (entry['filename'] != filename or
                    hashlib.sha256(Path(source.origin).read_bytes()).hexdigest() != entry['source_sha256'] or
                    hashlib.sha256((_QUERY_EXTENSIONS / filename).read_bytes()).hexdigest() != entry['binary_sha256']):
                return False
    except (OSError, ValueError, TypeError, KeyError, ImportError):
        return False

    previous_path = list(package.__path__)
    package.__path__ = [str(_QUERY_EXTENSIONS)] + previous_path
    try:
        for name in qualified:
            importlib.import_module(name)
    except (ImportError, OSError):
        # A matching manifest is not enough if this runtime cannot load the
        # artifact. No provider or model object has been unpickled yet.
        package.__path__ = previous_path
        for name, full_name in zip(_QN_MODULES, qualified):
            sys.modules.pop(full_name, None)
            package.__dict__.pop(name, None)
        return False
    return True


def main():
    stream='--stream' in sys.argv
    output=sys.stdout.buffer
    def send(response):
        data=pickle.dumps(response)
        if stream:output.write(struct.pack('!Q',len(data)))
        output.write(data);output.flush()
    try:
        with contextlib.redirect_stdout(sys.stderr):
            _enable_query_extensions()
        provider,arguments=pickle.load(sys.stdin.buffer)
        with contextlib.redirect_stdout(sys.stderr):
            result=provider(*arguments)
            if stream:
                for candidate in result:send((True,candidate))
                return
        response=(True,result)
    except Exception as error:
        response=('BUDGET' if stream and type(error).__name__=='PlanningBudgetExceeded' else False,str(error))
    send(response)


if __name__=='__main__':
    main()
