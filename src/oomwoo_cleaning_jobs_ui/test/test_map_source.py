from types import SimpleNamespace
import numpy as np
from oomwoo_cleaning_jobs_ui.map_source import source_map_from_occupancy_grid

def test_occupancy_grid_conversion_preserves_raw_data_and_yaw():
    message=SimpleNamespace(info=SimpleNamespace(width=3,height=2,resolution=0.1,
        origin=SimpleNamespace(position=SimpleNamespace(x=1.0,y=2.0),orientation=SimpleNamespace(x=0.0,y=0.0,z=1.0,w=0.0))),
        data=[0,10,-1,100,0,42])
    source=source_map_from_occupancy_grid(message)
    assert source.cells.dtype == np.int8
    assert source.cells.tolist()==[[0,10,-1],[100,0,42]]
    assert source.origin[0:2]==(1.0,2.0)
    assert abs(abs(source.origin[2])-np.pi)<1e-6
