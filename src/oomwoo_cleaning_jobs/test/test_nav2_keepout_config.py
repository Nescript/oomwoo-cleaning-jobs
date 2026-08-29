"""Guard tests for the Nav2 keepout filter configuration sample.

Costmap filters apply in plugin-list order; listing the keepout filter
before the inflation layer would let the inflation layer expand lethal
mask cells, silently changing constraint semantics. These tests lock the
documented ordering contract of config/nav2_keepout.yaml.
"""

from pathlib import Path

import yaml

CONFIG = Path(__file__).parent.parent / 'config' / 'nav2_keepout.yaml'


def _plugins(costmap: str) -> list[str]:
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    return data[costmap][costmap]['ros__parameters']['plugins']


def test_keepout_filter_is_listed_after_inflation_layer():
    for costmap in ('global_costmap', 'local_costmap'):
        plugins = _plugins(costmap)
        assert 'keepout_filter' in plugins, costmap
        assert 'inflation_layer' in plugins, costmap
        assert plugins.index('keepout_filter') > plugins.index('inflation_layer'), costmap


def test_keepout_filter_uses_standard_plugin_and_info_topic():
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    for costmap in ('global_costmap', 'local_costmap'):
        params = data[costmap][costmap]['ros__parameters']['keepout_filter']
        assert params['plugin'] == 'nav2_costmap_2d::KeepoutFilter'
        assert params['filter_info_topic'] == '/costmap_filter_info'
        assert params['override_lethal_cost'] is True
