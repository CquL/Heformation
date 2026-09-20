"""Compile the production patch against the vendored upstream implementation."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_normalization_degeneracy_keeps_real_roots(tmp_path):
    compiler=shutil.which('g++')
    if not compiler or not Path('/usr/include/eigen3/Eigen/Eigen').is_file():
        pytest.skip('native root regression requires g++ and Eigen headers')
    root=Path(__file__).resolve().parents[3]
    header=Path('src/planner/traj_opt/include/optimizer/root_finder.hpp')
    target=tmp_path/header
    target.parent.mkdir(parents=True)
    shutil.copyfile(root/'upstream/Swarm-Formation'/header,target)
    patch=root/'integration/swarm_qn_bridge/patches/root_finder_degenerate_bound.patch'
    subprocess.run(['patch','-p1','-d',str(tmp_path)],input=patch.read_bytes(),check=True,capture_output=True)
    source=tmp_path/'regression.cpp'
    source.write_text(r'''
#include <cassert>
#include <cmath>
#include "root_finder.hpp"
int main() {
    // x^6 - a^6 has exactly two real roots, at +/-a. The large-scale
    // polynomial used to leave an empty Kojima ratio vector after normalization.
    for (double scale : {1., 1e20}) {
        Eigen::VectorXd c=Eigen::VectorXd::Zero(7);
        c(0)=scale; c(6)=-1.;
        auto roots=RootFinder::solvePolynomial(c,-2.,2.,1e-9);
        double a=std::pow(1./scale,1./6.);
        assert(roots.size()==2);
        assert(std::abs(*roots.begin()+a)<1e-8);
        assert(std::abs(*roots.rbegin()-a)<1e-8);
    }
    // Ordinary well-conditioned sixth-degree case retains all six real roots.
    Eigen::VectorXd c(7); c << 1.,0.,-14.,0.,49.,0.,-36.;
    auto roots=RootFinder::solvePolynomial(c,-4.,4.,1e-9);
    assert(roots.size()==6);
    auto it=roots.begin();
    for (double expected : {-3.,-2.,-1.,1.,2.,3.})
        assert(std::abs(*it++-expected)<1e-8);
}
''')
    binary=tmp_path/'regression'
    subprocess.run([compiler,'-O2','-I/usr/include/eigen3','-I'+str(target.parent),
                    str(source),'-o',str(binary)],check=True,capture_output=True,timeout=60)
    subprocess.run([str(binary)],check=True,capture_output=True,timeout=10)
