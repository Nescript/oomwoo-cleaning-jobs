"""Test-only room segmenter used to isolate GUI tests from ROS providers."""

import numpy as np

from oomwoo_segmentation.models import SegmentationResult
from oomwoo_segmentation.validation import canonicalize_labels


def fake_segmentation(source, cleanable_mask=None):
    cleanable = source.free_mask()
    if cleanable_mask is not None:
        cleanable &= np.asarray(cleanable_mask, dtype=bool)
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    rows, cols = np.nonzero(cleanable)
    if rows.size:
        r0, r1 = int(rows.min()), int(rows.max())
        c0, c1 = int(cols.min()), int(cols.max())
        scores = source.occupied_mask()[r0:r1 + 1, c0:c1 + 1].sum(axis=0)
        offset = int(np.argmax(scores))
        wall_col = c0 + offset
        columns = np.indices(labels.shape)[1]
        if scores[offset] >= max(3, (r1 - r0 + 1) // 3):
            labels[cleanable & (columns < wall_col)] = 1
            labels[cleanable & (columns > wall_col)] = 2
        else:
            labels[cleanable] = 1
    canonical, regions, cleanable = canonicalize_labels(labels, source, cleanable)
    return SegmentationResult(canonical, regions, cleanable, 'test-fake', '1')
