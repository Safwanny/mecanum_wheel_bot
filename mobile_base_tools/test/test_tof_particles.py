# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ParticleMemory: merge, refresh, fade and expire."""

from mobile_base_tools.tof_particles import ParticleMemory


def test_merge_refresh_and_expire():
    memory = ParticleMemory(decay=5.0, merge_distance=0.01)
    memory.add([(1.0, 0.0, 0.03), (1.001, 0.0, 0.03)], now=0.0)
    assert len(memory.snapshot(0.0)) == 1          # merged into one
    memory.add([(2.0, 0.0, 0.03)], now=3.0)
    fresh = {round(p[0]): p[3] for p in memory.snapshot(4.0)}
    assert fresh[1] < fresh[2]                     # older fades first
    memory.add([(1.0, 0.0, 0.03)], now=5.5)        # refreshed, not expired
    assert len(memory.snapshot(7.0)) == 2
    assert [round(p[0]) for p in memory.snapshot(9.0)] == [1]  # 2 expired
    assert memory.snapshot(20.0) == []
