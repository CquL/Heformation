import importlib.util
from pathlib import Path
import random


def load_nearest():
    path=Path(__file__).parents[1]/'scripts/verify_formation_experiment.py'
    spec=importlib.util.spec_from_file_location('verification_nearest_under_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.nearest


def test_indexed_match_preserves_missing_windows_duplicates_and_ties():
    nearest=load_nearest();rng=random.Random(4)
    series=sorted([(rng.randrange(100)/10.,i) for i in range(300)])
    for stamp in [-1.,0.,.05,.5,9.9,10.,11.]:
        for window in [0.,.01,.05,.1,2.]:
            expected=min(series,key=lambda s:abs(s[0]-stamp))
            if abs(expected[0]-stamp)>window:expected=None
            assert nearest(series,stamp,window) is expected
    assert nearest([],1.,1.) is None


def test_lookup_does_not_scan_long_recording():
    class Series:
        reads=0
        def __len__(self):return 100000
        def __getitem__(self,index):
            self.reads+=1
            return (index/100.,index)
    rows=Series();result=load_nearest()(rows,500.004,.01)
    assert result[1]==50000
    assert rows.reads<60
