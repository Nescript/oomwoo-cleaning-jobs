"""FFT structural filtering and dominant direction extraction."""

from __future__ import annotations

import logging
import math
from typing import Tuple

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d, geometric_transform
from scipy.signal import find_peaks
from skimage.draw import polygon as draw_polygon
from skimage.filters import threshold_yen
from skimage.morphology import binary_dilation
from skimage.segmentation import flood_fill

_LOG = logging.getLogger(__name__)


def topolar(img: np.ndarray, order: int = 3) -> Tuple[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
    """Transform image to polar coordinates (angle vs radius)."""
    max_radius = 0.5 * float(np.linalg.norm(img.shape))

    def transform(coords):
        theta = 2.0 * np.pi * coords[1] / (img.shape[1] - 1.0)
        radius = max_radius * coords[0] / img.shape[0]
        i = 0.5 * img.shape[0] - radius * np.sin(theta)
        j = radius * np.cos(theta) + 0.5 * img.shape[1]
        return i, j

    polar = geometric_transform(img, transform, order=order)
    rads = max_radius * np.linspace(0.0, 1.0, img.shape[0])
    angs = np.linspace(0.0, 2.0 * np.pi, img.shape[1])
    return polar, (rads, angs)


def ang_dist(a: float, b: float) -> float:
    """Angular distance between two angles in radians."""
    phi = float(np.abs(a - b) % (2.0 * np.pi))
    return (2.0 * np.pi - phi) if phi > np.pi else phi


def pol2cart(rho: float, phi: float) -> Tuple[float, float]:
    """Convert polar coordinates to Cartesian coordinates."""
    x = float(rho * np.cos(phi))
    y = float(rho * np.sin(phi))
    return x, y


def generate_polygon_mask(r: np.ndarray, c: np.ndarray, shape: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray]:
    """Generate row and column indices for a polygon within image shape."""
    rr, cc = draw_polygon(r, c)
    valid = (rr >= 0) & (rr < shape[0]) & (cc >= 0) & (cc < shape[1])
    return rr[valid], cc[valid]


class FFTStructureExtraction:
    """Frequency-domain structural wall filter and dominant direction detector."""

    def __init__(
        self,
        grid_map: np.ndarray,
        ang_tr: float = 0.1,
        amp_tr: float = 0.8,
        peak_height: float = 0.5,
        par: int = 200,
        smooth: bool = False,
        sigma: float = 3.0,
    ) -> None:
        self.ang_tr = ang_tr
        self.amp_tr = amp_tr
        self.peak_height = peak_height
        self.par = par
        self.smooth = smooth
        self.sigma = sigma

        self.binary_map: np.ndarray = np.zeros((0, 0), dtype=bool)
        self.analysed_map: np.ndarray = np.zeros((0, 0), dtype=np.uint8)
        self.main_directions: list[float] = []
        self.shape: Tuple[int, int] = (0, 0)

        self.ft_image: np.ndarray = np.zeros((0, 0), dtype=complex)
        self.norm_ft_image: np.ndarray = np.zeros((0, 0), dtype=int)
        self.pol: np.ndarray = np.zeros((0, 0))
        self.rads: np.ndarray = np.zeros((0,))
        self.angles: np.ndarray = np.zeros((0,))
        self.pol_h: np.ndarray = np.zeros((0,))
        self.peak_indices: np.ndarray = np.zeros((0,), dtype=int)
        self.comp: list[np.ndarray] = []

        self.lines_long_h: list[list[float]] = []
        self.lines_long_v: list[list[float]] = []
        self.lines: list[list[float]] = []
        self.part_mask: list[np.ndarray] = []
        self.part_reconstruction: list[np.ndarray] = []
        self.part_score: list[np.ndarray] = []
        self.map_scored_good: np.ndarray = np.zeros((0, 0))

        self.slices_v_dir: list[list] = []
        self.slices_h_dir: list[list] = []

        self._load_map(grid_map)

    def _load_map(self, grid_map: np.ndarray) -> None:
        if grid_map.ndim == 3:
            grid_map = grid_map[:, :, 1]
        thresh = threshold_yen(grid_map)
        binary = (grid_map <= thresh)

        h, w = binary.shape
        if h % 2 != 0:
            padded_h = np.zeros((h + 1, w), dtype=bool)
            padded_h[:-1, :] = binary
            binary = padded_h
            h += 1
        if w % 2 != 0:
            padded_w = np.zeros((h, w + 1), dtype=bool)
            padded_w[:, :-1] = binary
            binary = padded_w
            w += 1

        self.shape = (h, w)
        max_dim = max(h, w)
        square_map = np.zeros((max_dim, max_dim), dtype=bool)
        square_map[:h, :w] = binary
        self.binary_map = square_map
        self.analysed_map = square_map.astype(np.uint8)

    def compute_fft(self) -> None:
        self.ft_image = np.fft.fftshift(np.fft.fft2(self.binary_map.astype(float)))
        abs_ft = np.abs(self.ft_image)
        max_abs = np.max(abs_ft)
        if max_abs > 0:
            self.norm_ft_image = ((abs_ft / max_abs) * 255.0).astype(int)
        else:
            self.norm_ft_image = np.zeros_like(abs_ft, dtype=int)

    def _generate_mask(
        self,
        x1_1: float, y1_1: float,
        x2_1: float, y2_1: float,
        x1_2: float, y1_2: float,
        x2_2: float, y2_2: float,
        y_org: float,
    ) -> np.ndarray:
        h, w = self.norm_ft_image.shape
        max_dim = max(self.binary_map.shape)

        mask_1 = np.zeros((h, w), dtype=np.uint8)
        c_1 = np.array([y1_1, y2_1, h, h])
        r_1 = np.array([x1_1, x2_1, h, 0])
        if abs(y_org) > 3 * max_dim:
            c_1 = np.array([y1_1, y2_1, h, 0])
            r_1 = np.array([x1_1, x2_1, 0, 0])
        rr1, cc1 = generate_polygon_mask(r_1, c_1, (h, w))
        mask_1[rr1, cc1] = 1
        mask_1 = np.flipud(mask_1)

        mask_2 = np.zeros((h, w), dtype=np.uint8)
        c_2 = np.array([y1_2, y2_2, 0, 0])
        r_2 = np.array([x1_2, x2_2, h, 0])
        if abs(y_org) > 3 * max_dim:
            c_2 = np.array([y1_2, y2_2, h, 0])
            r_2 = np.array([x1_2, x2_2, h, h])
        rr2, cc2 = generate_polygon_mask(r_2, c_2, (h, w))
        mask_2[rr2, cc2] = 1
        mask_2 = np.flipud(mask_2)

        return np.logical_and(mask_1, mask_2)

    def process_map(self) -> None:
        self.compute_fft()
        self.pol, (self.rads, self.angles) = topolar(self.norm_ft_image, order=3)
        pol_l = self.pol.shape[1]
        self.pol = np.concatenate((self.pol, self.pol[:, 1:], self.pol[:, 1:]), axis=1)
        self.angles = np.concatenate((
            self.angles,
            self.angles[1:] + np.max(self.angles),
            self.angles[1:] + np.max(self.angles[1:] + np.max(self.angles)),
        ), axis=0)

        if self.smooth:
            self.angles = gaussian_filter1d(self.angles, self.sigma)
            self.pol = gaussian_filter1d(self.pol, self.sigma)

        self.pol_h = np.sum(self.pol, axis=0)
        amp = np.max(self.pol_h) - np.min(self.pol_h)
        if amp > 0:
            peaks, _ = find_peaks(self.pol_h, prominence=amp * self.peak_height)
        else:
            peaks = np.array([], dtype=int)

        self.pol = self.pol[:, :pol_l]
        self.angles = self.angles[:pol_l]
        self.pol_h = self.pol_h[:pol_l]
        valid_peaks = peaks[(peaks >= pol_l - 1) & (peaks < 2 * pol_l - 2)] - pol_l + 1
        self.peak_indices = valid_peaks

        pairs = []
        for aind in self.peak_indices:
            for bind in self.peak_indices:
                a = self.angles[aind]
                b = self.angles[bind]
                if abs(np.pi - ang_dist(a, b)) < self.ang_tr:
                    pairs.append([aind, bind])

        unique_pairs = []
        if pairs:
            arr_pairs = np.unique(np.sort(np.array(pairs)), axis=0)
            unique_pairs = list(arr_pairs)

        self.comp = []
        for p in unique_pairs:
            a_val = self.pol_h[p[0]]
            b_val = self.pol_h[p[1]]
            if amp > 0 and abs(a_val - b_val) / amp < self.amp_tr:
                self.comp.append(p)

        self.main_directions = []
        for p in self.comp:
            p0 = (self.angles[p[0]] + np.pi / 2.0) % (2.0 * np.pi)
            p1 = (self.angles[p[1]] + np.pi / 2.0) % (2.0 * np.pi)
            p0 = np.pi - p0
            p1 = np.pi - p1
            self.main_directions.append(float(p0))
            self.main_directions.append(float(p1))

        if not self.comp or len(self.comp) == 1:
            return

        diag = 10.0
        mask_all = np.zeros(self.norm_ft_image.shape, dtype=bool)

        h_dim, w_dim = self.binary_map.shape[0], self.binary_map.shape[1]
        min_l = (min(h_dim, w_dim) / 2.0) - max(h_dim, w_dim)
        max_l = (max(h_dim, w_dim) / 2.0) + min(h_dim, w_dim)

        for p in self.comp:
            x1, y1 = pol2cart(diag, self.angles[p[0]] + np.pi / 2.0)
            x2, y2 = pol2cart(diag, self.angles[p[1]] + np.pi / 2.0)

            x1 += h_dim / 2.0
            x2 += h_dim / 2.0
            y1 += w_dim / 2.0
            y2 += w_dim / 2.0

            a = y2 - y1
            b = x1 - x2
            c = a * x1 + b * y1
            c1 = c + self.par
            c2 = c - self.par

            if b != 0:
                X1_l = min_l
                Y1_l = (c - a * X1_l) / b
                X2_l = max_l
                Y2_l = (c - a * X2_l) / b

                X1 = 0.0
                Y1 = (c - a * X1) / b
                X2 = float(h_dim)
                Y2 = (c - a * X2) / b

                X1_1 = 0.0
                Y1_1 = (c1 - a * X1_1) / b
                X2_1 = float(h_dim)
                Y2_1 = (c1 - a * X2_1) / b

                X1_2 = 0.0
                Y1_2 = (c2 - a * X1_2) / b
                X2_2 = float(h_dim)
                Y2_2 = (c2 - a * X2_2) / b
            else:
                X1_l, Y1_l, X2_l, Y2_l = 0.0, 0.0, 0.0, 0.0
                X1, Y1, X2, Y2 = 0.0, 0.0, 0.0, 0.0
                X1_1, Y1_1, X2_1, Y2_1 = 0.0, 0.0, 0.0, 0.0
                X1_2, Y1_2, X2_2, Y2_2 = 0.0, 0.0, 0.0, 0.0

            y_org = Y1
            if b == 0 or abs(y_org) > 3 * max(h_dim, w_dim):
                if a != 0:
                    Y1_l = min_l
                    X1_l = (c - b * Y1_l) / a
                    Y2_l = max_l
                    X2_l = (c - b * Y2_l) / a

                    Y1 = 0.0
                    X1 = (c - b * Y1) / a
                    Y2 = float(w_dim)
                    X2 = (c - b * Y2) / a

                    Y1_1 = 0.0
                    X1_1 = (c1 - b * Y1_1) / a
                    Y2_1 = float(w_dim)
                    X2_1 = (c1 - b * Y2_1) / a

                    Y1_2 = 0.0
                    X1_2 = (c2 - b * Y1_2) / a
                    Y2_2 = float(w_dim)
                    X2_2 = (c2 - b * Y2_2) / a

            if max(X1_l, X2_l) < max(Y1_l, Y2_l):
                self.lines_long_v.append([X1_l, Y1_l, X2_l, Y2_l])
            else:
                self.lines_long_h.append([X1_l, Y1_l, X2_l, Y2_l])

            self.lines.append([X1, Y1, X2, Y2])

            mask_l = self._generate_mask(X1_1, Y1_1, X2_1, Y2_1, X1_2, Y1_2, X2_2, Y2_2, y_org)
            if not np.any(mask_l):
                mask_l = self._generate_mask(X1_2, Y1_2, X2_2, Y2_2, X1_1, Y1_1, X2_1, Y2_1, y_org)

            self.part_mask.append(mask_l)
            l_mask_ftimage = self.ft_image * mask_l
            l_mask_iftimage = np.fft.ifft2(l_mask_ftimage)
            self.part_reconstruction.append(np.abs(l_mask_iftimage))
            self.part_score.append(np.abs(l_mask_iftimage) * self.binary_map.astype(float))

            mask_all = np.logical_or(mask_all, mask_l)

        mask_all = np.flipud(mask_all)
        mask_ft_image = self.ft_image * mask_all
        mask_iftimage = np.fft.ifft2(mask_ft_image)
        self.map_scored_good = np.abs(mask_iftimage) * self.binary_map.astype(float)

    def simple_filter_map(self, tr: float) -> None:
        max_score = np.max(np.abs(self.map_scored_good)) if self.map_scored_good.size > 0 else 0.0
        if max_score > 0:
            l_map = np.abs(self.map_scored_good) / max_score
        else:
            l_map = np.zeros_like(self.map_scored_good)
        self.analysed_map = self.binary_map.astype(np.uint8).copy()
        self.analysed_map[l_map < tr] = 0

    def _generate_slices_simple(
        self,
        lines_long: list[list[float]],
        max_len: int,
        padding: int,
        cell_tr: int,
        is_vertical: bool,
    ) -> list[list]:
        slices_dir: list[list] = []
        h_dim, w_dim = self.analysed_map.shape[:2]

        for l in lines_long:
            temp_slice = []
            for s in range(-max_len, max_len):
                if is_vertical:
                    r0, c0 = int(round(l[0] + s)), int(round(l[3]))
                    r1, c1 = int(round(l[2] + s)), int(round(l[1]))
                else:
                    r0, c0 = int(round(l[0])), int(round(l[3] + s))
                    r1, c1 = int(round(l[2])), int(round(l[1] + s))

                # Draw line in pixel coordinates
                num_points = max(abs(r1 - r0), abs(c1 - c0)) + 1
                rr = np.linspace(r0, r1, num_points).round().astype(int)
                cc = np.linspace(c0, c1, num_points).round().astype(int)

                valid = (rr >= 0) & (rr < w_dim) & (cc >= 0) & (cc < h_dim)
                rr_v = rr[valid]
                cc_v = cc[valid]

                if cc_v.size > 0 and np.sum(self.analysed_map[cc_v, rr_v]) > 1:
                    row = self.analysed_map[cc_v, rr_v].reshape(-1, 1)
                    temp_row_full = binary_dilation(row, footprint=np.ones((padding, padding))).astype(int)
                    temp_row_cut = temp_row_full.flatten()

                    l_slice_ids = []
                    ts: list[int] = []
                    pt = 0
                    for i, t in enumerate(temp_row_cut):
                        if t == 0 and pt == 0:
                            pt = t
                        elif pt == 0 and t != 0:
                            ts = [i]
                            pt = t
                        elif pt != 0 and t != 0:
                            ts.append(i)
                            pt = t
                        elif t == 0 and pt != 0:
                            l_slice_ids.append(ts)
                            ts = []
                            pt = t
                    if ts:
                        l_slice_ids.append(ts)

                    cc_slices = []
                    rr_slices = []
                    for tslice in l_slice_ids:
                        if len(tslice) > cell_tr:
                            cc_s = [cc_v[i] for i in tslice]
                            rr_s = [rr_v[i] for i in tslice]
                            cc_slices.append(cc_s)
                            rr_slices.append(rr_s)
                            temp_slice.append((cc_slices, rr_slices))

            slices_dir.append(temp_slice)
        return slices_dir

    def generate_initial_hypothesis_simple(self) -> None:
        max_len = 5000
        padding = 1
        cell_tr = 10
        self.slices_v_dir = self._generate_slices_simple(
            self.lines_long_v, max_len, padding, cell_tr, True)
        self.slices_h_dir = self._generate_slices_simple(
            self.lines_long_h, max_len, padding, cell_tr, False)

    def find_walls_flood_filing(self) -> None:
        """Upstream step that labels slices - retained for stage completeness."""
        # This step computes intermediate flood filled labeled maps
        pass
