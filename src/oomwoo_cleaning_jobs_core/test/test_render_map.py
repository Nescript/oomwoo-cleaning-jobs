"""Tests for render / the render_map CLI."""

import numpy as np

from fixtures import make_rooms_map, make_two_rooms_map, write_map_files

from oomwoo_cleaning_jobs_core.render import (
    COLOR_FREE,
    COLOR_OCCUPIED,
    COLOR_UNKNOWN,
    render_segmentation,
    render_source_map,
)
from oomwoo_cleaning_jobs_core.render_map import main
from oomwoo_cleaning_jobs_core.segmentation import segment


def test_render_source_map_colors_and_orientation():
    source = make_rooms_map()
    img = render_source_map(source)
    assert img.shape == (source.height, source.width, 3)
    # color counts match the cell classification counts
    for color, count in (
        (COLOR_FREE, source.free_mask().sum()),
        (COLOR_OCCUPIED, source.occupied_mask().sum()),
        (COLOR_UNKNOWN, source.unknown_mask().sum()),
    ):
        assert (img == color).all(axis=2).sum() == count
    # vertical flip: cells row 0 (bottom row) must be the last image row
    free_rows = np.argwhere(source.free_mask().any(axis=1)).ravel()
    bottom_row = free_rows[0]
    assert (img[source.height - 1 - bottom_row] == COLOR_FREE).all(axis=1).any()


def test_render_segmentation_smoke():
    source = make_rooms_map()
    result = segment(source)
    img = render_segmentation(source, result, scale=2)
    assert img.shape == (source.height * 2, source.width * 2, 3)
    # every region color appears in the image
    from oomwoo_cleaning_jobs_core.render import region_color
    for region in result.regions:
        color = region_color(region.label)
        # the overlay is alpha-blended (largest deviation ~180 for low-saturation
        # colors blended with the white background)
        diff = np.abs(img.astype(int) - np.array(color)).sum(axis=2)
        assert (diff < 200).any()


def test_cli_base_render(tmp_path, capsys):
    yaml_path = write_map_files(tmp_path, make_rooms_map())
    assert main([str(yaml_path)]) == 0
    out = tmp_path / 'map.render.png'
    assert out.exists()
    stdout = capsys.readouterr().out
    source = make_rooms_map()
    assert source.identity in stdout
    assert source.short_id in stdout


def test_cli_with_segmentation(tmp_path, capsys):
    yaml_path = write_map_files(tmp_path, make_two_rooms_map())
    assert main([str(yaml_path), '--segment']) == 0
    assert (tmp_path / 'map.render.png').exists()
    assert (tmp_path / 'map.segments.png').exists()
    stdout = capsys.readouterr().out
    assert 'candidates : 2' in stdout
