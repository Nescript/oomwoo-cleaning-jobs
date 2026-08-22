"""map_io 加载器测试：nav2 trinary 保真度。"""

import cv2
import numpy as np
import pytest
import yaml

from fixtures import PIXEL_FREE, PIXEL_OCCUPIED, PIXEL_UNKNOWN, make_rooms_map, write_map_files

from oomwoo_cleaning_jobs_core import FREE, OCCUPIED, UNKNOWN, load_map_file


def _write_custom_map(tmp_path, image, meta_overrides=None, name='custom'):
    image_path = tmp_path / f'{name}.png'
    assert cv2.imwrite(str(image_path), image)
    meta = {
        'image': image_path.name,
        'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'mode': 'trinary',
    }
    meta.update(meta_overrides or {})
    yaml_path = tmp_path / f'{name}.yaml'
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f)
    return yaml_path


def test_round_trip_saver_convention(tmp_path):
    source = make_rooms_map()
    loaded = load_map_file(write_map_files(tmp_path, source))
    assert loaded == source


def test_pixel_classification_default_thresholds(tmp_path):
    # 一列一个像素；默认阈值 0.65/0.25 下：
    # 0 → occ 1.0 → 100；89 → 0.651 → 100；128 → 0.498 → -1；
    # 200 → 0.216 → 0；240 → 0.059 → 0；255 → 0
    image = np.array([[0], [89], [128], [200], [240], [255]], dtype=np.uint8)
    loaded = load_map_file(_write_custom_map(tmp_path, image))
    col = loaded.cells[:, 0].tolist()
    # cells 行序 = 图像上下翻转
    assert col == [FREE, FREE, FREE, UNKNOWN, OCCUPIED, OCCUPIED]


def test_vertical_flip(tmp_path):
    image = np.full((4, 2), PIXEL_FREE, dtype=np.uint8)
    image[0, :] = PIXEL_OCCUPIED  # 图像顶行 = 地图最大 y
    loaded = load_map_file(_write_custom_map(tmp_path, image))
    assert loaded.cells[-1, :].tolist() == [OCCUPIED, OCCUPIED]
    assert loaded.cells[0, :].tolist() == [FREE, FREE]


def test_unknown_pixels_round_trip(tmp_path):
    image = np.full((3, 3), PIXEL_UNKNOWN, dtype=np.uint8)
    loaded = load_map_file(_write_custom_map(
        tmp_path, image, {'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    assert (loaded.cells == UNKNOWN).all()


def test_alpha_below_255_is_unknown(tmp_path):
    image = np.full((2, 2, 4), 255, dtype=np.uint8)  # 白色
    image[0, 0, 3] = 0  # 透明 → unknown
    loaded = load_map_file(_write_custom_map(tmp_path, image))
    assert loaded.cells[-1, 0] == UNKNOWN
    assert loaded.cells[-1, 1] == FREE


def test_non_trinary_mode_rejected(tmp_path):
    image = np.full((2, 2), PIXEL_FREE, dtype=np.uint8)
    yaml_path = _write_custom_map(tmp_path, image, {'mode': 'scale'})
    with pytest.raises(ValueError, match='trinary'):
        load_map_file(yaml_path)


def test_missing_required_field(tmp_path):
    image = np.full((2, 2), PIXEL_FREE, dtype=np.uint8)
    yaml_path = _write_custom_map(tmp_path, image)
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    del meta['resolution']
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f)
    with pytest.raises(ValueError, match='resolution'):
        load_map_file(yaml_path)


def test_metadata_preserved(tmp_path):
    source = make_rooms_map()
    loaded = load_map_file(write_map_files(tmp_path, source))
    assert loaded.resolution == pytest.approx(0.05)
    assert loaded.origin == (-2.5, -2.0, 0.0)
    assert (loaded.width, loaded.height) == (100, 80)
