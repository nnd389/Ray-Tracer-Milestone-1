#!/usr/bin/env python3

import re
import sys
import os
import shutil
import subprocess
import argparse
import colorama
import hashlib
from colorama import Fore, Style
import numpy as np
from math import sqrt
from PIL import Image
import matplotlib.pyplot as plt
import functools
from scipy.ndimage import uniform_filter
from pathlib import Path
import signal


def _msg(text, level, color):
    return Style.BRIGHT + color + level + Fore.RESET + Style.NORMAL + text


def _info(text):
    print(_msg(text, "[INFO]:  ", Fore.BLUE))


def _warn(text):
    print(_msg(text, "[WARN]:  ", Fore.YELLOW))


def _error(text):
    print(_msg(text, "[ERROR]: ", Fore.RED))


def _fatal(text):
    print(_msg(text, "[FATAL]: ", Fore.RED))
    sys.exit(1)


# The code from here up to the comment which says "ENDSCIKITIMG" is taken from
# the scikit-image library v0.22.x and modified under the BSD 3-Clause
# License. https://github.com/scikit-image/scikit-image/tree/v0.22.x

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the University nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HOLDERS OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


def slice_at_axis(sl, axis):
    return (slice(None),) * axis + (sl,) + (...,)


def crop(ar, crop_width, copy=False, order="K"):
    ar = np.array(ar, copy=False)
    crops = [[crop_width, crop_width]] * ar.ndim

    slices = tuple(slice(a, ar.shape[i] - b) for i, (a, b) in enumerate(crops))
    if copy:
        cropped = np.array(ar[slices], order=order, copy=True)
    else:
        cropped = ar[slices]
    return cropped


def structural_similarity(
    im1,
    im2,
    *,
    win_size=None,
    gradient=False,
    data_range=None,
    channel_axis=None,
    full=False,
    **kwargs,
):
    """
    Compute the mean structural similarity index between two images.
    Please pay attention to the `data_range` parameter with floating-point images.

    Parameters
    ----------
    im1, im2 : ndarray
        Images. Any dimensionality with same shape.
    win_size : int or None, optional
        The side-length of the sliding window used in comparison. Must be an
        odd value.
    gradient : bool, optional
        If True, also return the gradient with respect to im2.
    data_range : float, optional
        The data range of the input image (distance between minimum and
        maximum possible values). By default, this is estimated from the image
        data type. This estimate may be wrong for floating-point image data.
        Therefore it is recommended to always pass this value explicitly
        (see note below).
    channel_axis : int or None, optional
        If None, the image is assumed to be a grayscale (single channel) image.
        Otherwise, this parameter indicates which axis of the array corresponds
        to channels.

        .. versionadded:: 0.19
           ``channel_axis`` was added in 0.19.
    full : bool, optional
        If True, also return the full structural similarity image.

    Other Parameters
    ----------------
    use_sample_covariance : bool
        If True, normalize covariances by N-1 rather than, N where N is the
        number of pixels within the sliding window.
    K1 : float
        Algorithm parameter, K1 (small constant, see [1]_).
    K2 : float
        Algorithm parameter, K2 (small constant, see [1]_).

    Returns
    -------
    mssim : float
        The mean structural similarity index over the image.
    grad : ndarray
        The gradient of the structural similarity between im1 and im2 [2]_.
        This is only returned if `gradient` is set to True.
    S : ndarray
        The full SSIM image.  This is only returned if `full` is set to True.

    .. versionchanged:: 0.16
        This function was renamed from ``skimage.measure.compare_ssim`` to
        ``skimage.metrics.structural_similarity``.

    References
    ----------
    .. [1] Wang, Z., Bovik, A. C., Sheikh, H. R., & Simoncelli, E. P.
       (2004). Image quality assessment: From error visibility to
       structural similarity. IEEE Transactions on Image Processing,
       13, 600-612.
       https://ece.uwaterloo.ca/~z70wang/publications/ssim.pdf,
       :DOI:`10.1109/TIP.2003.819861`

    .. [2] Avanaki, A. N. (2009). Exact global histogram specification
       optimized for structural similarity. Optical Review, 16, 613-621.
       :arxiv:`0901.0065`
       :DOI:`10.1007/s10043-009-0119-z`

    """
    assert im1.shape == im2.shape, "Input images must have the same dimensions."
    assert im1.dtype == np.float32, "Input images must have dtype float32."
    assert im2.dtype == np.float32, "Input images must have dtype float32."
    assert data_range is not None, "Specify data_range."

    if channel_axis is not None:
        # loop over channels
        args = dict(
            win_size=win_size,
            gradient=gradient,
            data_range=data_range,
            channel_axis=None,
            full=full,
        )
        args.update(kwargs)
        nch = im1.shape[channel_axis]
        mssim = np.empty(nch, dtype=np.float32)

        if gradient:
            G = np.empty(im1.shape, dtype=np.float32)
        if full:
            S = np.empty(im1.shape, dtype=np.float32)
        channel_axis = channel_axis % im1.ndim
        _at = functools.partial(slice_at_axis, axis=channel_axis)
        for ch in range(nch):
            ch_result = structural_similarity(im1[_at(ch)], im2[_at(ch)], **args)
            if gradient and full:
                mssim[ch], G[_at(ch)], S[_at(ch)] = ch_result
            elif gradient:
                mssim[ch], G[_at(ch)] = ch_result
            elif full:
                mssim[ch], S[_at(ch)] = ch_result
            else:
                mssim[ch] = ch_result
        mssim = mssim.mean()
        if gradient and full:
            return mssim, G, S
        elif gradient:
            return mssim, G
        elif full:
            return mssim, S
        else:
            return mssim

    K1 = kwargs.pop("K1", 0.01)
    K2 = kwargs.pop("K2", 0.03)
    sigma = kwargs.pop("sigma", 1.5)
    if K1 < 0:
        raise ValueError("K1 must be positive")
    if K2 < 0:
        raise ValueError("K2 must be positive")
    if sigma < 0:
        raise ValueError("sigma must be positive")
    use_sample_covariance = kwargs.pop("use_sample_covariance", True)

    if win_size is None:
        win_size = 7  # backwards compatibility

    if np.any((np.asarray(im1.shape) - win_size) < 0):
        raise ValueError(
            "win_size exceeds image extent. "
            "Either ensure that your images are "
            "at least 7x7; or pass win_size explicitly "
            "in the function call, with an odd value "
            "less than or equal to the smaller side of your "
            "images. If your images are multichannel "
            "(with color channels), set channel_axis to "
            "the axis number corresponding to the channels."
        )

    if not (win_size % 2 == 1):
        raise ValueError("Window size must be odd.")

    ndim = im1.ndim

    filter_func = uniform_filter
    filter_args = {"size": win_size}
    # ndimage filters need floating point data
    im1 = im1.astype(np.float32, copy=False)
    im2 = im2.astype(np.float32, copy=False)

    NP = win_size**ndim

    # filter has already normalized by NP
    if use_sample_covariance:
        cov_norm = NP / (NP - 1)  # sample covariance
    else:
        cov_norm = 1.0  # population covariance to match Wang et. al. 2004

    # compute (weighted) means
    ux = filter_func(im1, **filter_args)
    uy = filter_func(im2, **filter_args)

    # compute (weighted) variances and covariances
    uxx = filter_func(im1 * im1, **filter_args)
    uyy = filter_func(im2 * im2, **filter_args)
    uxy = filter_func(im1 * im2, **filter_args)
    vx = cov_norm * (uxx - ux * ux)
    vy = cov_norm * (uyy - uy * uy)
    vxy = cov_norm * (uxy - ux * uy)

    R = data_range
    C1 = (K1 * R) ** 2
    C2 = (K2 * R) ** 2

    A1, A2, B1, B2 = (
        2 * ux * uy + C1,
        2 * vxy + C2,
        ux**2 + uy**2 + C1,
        vx + vy + C2,
    )
    D = B1 * B2
    S = (A1 * A2) / D

    # to avoid edge effects will ignore filter radius strip around edges
    pad = (win_size + 1) // 2

    # compute (weighted) mean of ssim. Use float64 for accuracy.
    mssim = crop(S, pad).mean(dtype=np.float64)

    # Pad the regions that experience edge effects so that we don't get
    # bizarre funky output in the diff/montage files.
    w, h = S.shape
    S[:, :pad] = 1.0
    S[:, h - pad :] = 1.0
    S[:pad, :] = 1.0
    S[w - pad :, :] = 1.0

    if gradient:
        # The following is Eqs. 7-8 of Avanaki 2009.
        grad = filter_func(A1 / D, **filter_args) * im1
        grad += filter_func(-S / B2, **filter_args) * im2
        grad += filter_func((ux * (A2 - A1) - uy * (B2 - B1) * S) / D, **filter_args)
        grad *= 2 / im1.size

        if full:
            return mssim, grad, S
        else:
            return mssim, grad
    else:
        if full:
            return mssim, S
        else:
            return mssim


# ENDSCIKITIMG


def compute_rmsd(img1, img2):
    """Compute root-mean-square-distance between two images"""

    assert img1.shape == img2.shape, "Input images must have the same dimensions."
    assert img1.dtype == np.float32, "Input images must have dtype float32."
    assert img2.dtype == np.float32, "Input images must have dtype float32."

    delta = img1 - img2
    return np.linalg.norm(delta) / sqrt(delta.size)


def load_image(fn):
    """Read image into an RGB numpy array using PIL, also obtaining its max value"""

    img = np.array(Image.open(fn), dtype=np.float32)
    if img.ndim == 2:
        # Convert grayscale to RGB
        img = np.dstack((img, img, img))

    img = img / 255

    return img


def check_dirs(dirs):
    # print('checking dirs {}'.format(dirs))
    for d in dirs:
        if not d:
            continue
        if not os.path.exists(d):
            _error("{} does not exist".format(d))
            return False
        if not os.path.isdir(d):
            _error("{} is not a directory".format(d))
            return False
    return True


def check_files(files):
    # print('checking files {}'.format(files))
    for f in files:
        if not f:
            continue
        if not os.path.exists(f):
            _error("{} does not exist".format(f))
            return False
        if not os.path.isfile(f):
            _error("{} is not a file".format(f))
            return False
    return True


def hashfile(sha, path):
    BUF_SIZE = 32 * 1024
    with open(path, "rb") as f:
        while True:
            data = f.read(BUF_SIZE)
            if not data:
                break
            sha.update(data)


def check_ref_signature(refcache_dir, refbin, json, cubemap):
    """Hashes the reference binary together with the json/cubemap args used to
    generate the reference images to ensure that the reference directory is
    up-to-date. If it is not, we flush the cache."""
    sha = hashlib.sha256()
    signature_path = os.path.join(refcache_dir, "signature")
    hashfile(sha, refbin)
    if json:
        hashfile(sha, json)
    if cubemap:
        hashfile(sha, cubemap)
    need_flush = False
    need_update = False
    new_hash = sha.hexdigest().encode()
    if not os.path.exists(signature_path):
        # print("{} not exists, creating".format(signature_path))
        signature = new_hash
        need_update = True
    else:
        with open(signature_path, "rb") as f:
            signature = f.readline()
        # print("Hexing was {} expect {}".format(signature, new_hash))
        if signature != new_hash:
            # print("{} != {}".format(signature, new_hash))
            signature = new_hash
            need_flush = True
            need_update = True
    if need_flush:
        _info("Refcache {} is out of date, flushing".format(refcache_dir))
        shutil.rmtree(refcache_dir)
    os.makedirs(refcache_dir, exist_ok=True)
    if need_update:
        with open(signature_path, "wb") as f:
            f.write(signature)
    return need_update


def raycheck(args):
    refcache_dir = (
        os.path.join(args.out, "refcache") if not args.refcache else args.refcache
    )
    diffdir = os.path.join(args.out, "diff")
    mtgdir = os.path.join(args.out, "montage")
    scene_dir = args.scenes

    os.makedirs(args.out, exist_ok=True)
    if not check_dirs([scene_dir, args.out]) or (
        args.exec and not check_files([args.exec, args.refbin])
    ):
        return

    cmd_args = ["-r", "5"]
    if args.json:
        cmd_args += ["-j", args.json]
    if args.cubemap:
        cmd_args += ["-c", args.cubemap]
    if args.refbin:
        check_ref_signature(refcache_dir, args.refbin, args.json, args.cubemap)

    os.makedirs(os.path.join(args.out, "image"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "stdio"), exist_ok=True)
    os.makedirs(refcache_dir, exist_ok=True)
    os.makedirs(diffdir, exist_ok=True)
    os.makedirs(mtgdir, exist_ok=True)

    def run_one_raycheck(fn, root):
        if not fn.endswith(".json"):
            return

        # Flatten the directory structure: directory structure is nice for test
        # inputs, but makes it much harder to view all the outputs. By
        # flattening, we can scroll through all montages with an image viewer.
        # This is not robust against naughty test naming, but anyone who names a
        # test something like polymesh_dragon3 only has themselves to blame.
        rayfn = os.path.join(root, fn)
        relbase, _ = os.path.splitext(rayfn)
        relbase_parts = Path(os.path.relpath(relbase, start=scene_dir)).parts
        outstem = "_".join(relbase_parts)
        imagefn = os.path.join(args.out, "image", outstem + ".png")
        refimagefn = os.path.join(refcache_dir, outstem + ".std.png")
        stdoutfn = os.path.join(args.out, "stdio", outstem + ".out")
        stderrfn = os.path.join(args.out, "stdio", outstem + ".err")

        if args.refbin and not os.path.exists(refimagefn):
            alist = [args.refbin] + cmd_args + [rayfn, refimagefn]
            _info("executing: {}".format(" ".join(alist)))
            subprocess.run(alist)

        if args.exec:
            with open(stdoutfn, "w") as stdout, open(stderrfn, "w") as stderr:
                alist = [args.exec] + cmd_args + [rayfn, imagefn]
                _info("Running command: {}".format(" ".join(alist)))
                try:
                    subprocess.run(
                        alist, stdout=stdout, stderr=stderr, timeout=args.timelimit
                    )
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except subprocess.TimeoutExpired:
                    _error(outstem + " Timeout")
                except:
                    _error(outstem + " (Unknown)")

        return (imagefn, refimagefn, outstem)

    report = []
    try:
        for root, _, files in os.walk(scene_dir):
            for fn in files:
                result = run_one_raycheck(fn, root)
                if result is None or args.nocompare:
                    continue

                (imagefn, refimagefn, outstem) = result

                try:
                    img1 = load_image(imagefn)
                except Exception as e:
                    _error("Failed to load image {} because {}".format(imagefn, e))
                    report.append((outstem, 1.0, 0.0))
                    continue

                try:
                    img2 = load_image(refimagefn)
                except:
                    _fatal(
                        "Failed to load reference image {}, something is broken".format(
                            refimagefn
                        )
                    )

                assert img1.shape == img2.shape
                assert img1.shape[2] == 3

                rmsd = compute_rmsd(img1, img2)
                ssim, ssim_img = structural_similarity(
                    img1, img2, data_range=1.0, full=True, channel_axis=2
                )
                difffn = os.path.join(diffdir, outstem + ".diff.png")
                ssim_img = np.uint8(ssim_img * 255)
                ssim_img = Image.fromarray(ssim_img)
                ssim_img.save(difffn)

                mtgfn = os.path.join(mtgdir, outstem + ".mtg.png")
                subprocess.run(
                    [
                        "montage",
                        imagefn,
                        refimagefn,
                        difffn,
                        "-tile",
                        "3x",
                        "-geometry",
                        "+1+1",
                        mtgfn,
                    ]
                )
                rmsd = round(rmsd, 6)
                ssim = round(ssim, 6)
                report.append((outstem, rmsd, ssim))
    except KeyboardInterrupt:
        _info("Received Keyboard Interrupt, writing existing results to file")

    if report:
        lecture = """
This file reports differences between the images created by your raytracer and
those created by the reference raytracer. These results are in a comma-separated
value format. The columns are

test-name, ssim-value, rmsd-value

RMSD is the root-mean-squared-difference, a measure of difference between 
two images. Two identical images have an RMSD of 0, and it increases to a
max of 1.0 for maximally different images.

SSIM is the mean structural-similarity index. This is a simple statistical
metric of how perceptually-similar two images are. It is a number between 0 
and 1, where two identical images have an SSIM of 1.

No single image difference metric is perfect, and Whitted-style raytracers
without global illumination (including the reference raytracer) do not have a
single correct solution. There is no cutoff value past which an image is
unambiguously "correct". Do not use these numbers alone to decide whether your
raytracer is producing correct output.

You can view visualizations of the SSIM error in the montage directory. Once you
believe your raytracer is correctly rendering a scene (which will often be at
nonzero error values for complex scenes), you may copy the line for that test
from this file into a file called cutoffs.csv, located in the same directory as
raycheck.py.

raycheck.py will use cutoffs.csv to determine whether a test has regressed. If
the SSIM or RMSD value for a test is worse than the cutoff stored in
cutoffs.csv, raycheck.py will print a warning. You can use this to quickly
determine if your raytracer is failing scenes that it was previously rendering
correctly.

To suppress this message, pass the --i-understand-that-image-metrics-are-not-perfect
flag to the raycheck script.\n\n
"""
        with open(os.path.join(args.out, "report.csv"), "w") as f:
            if not args.suppress_memo:
                f.write(lecture)

            report_sorted = sorted(report, key=lambda x: x[2])

            for name, rmsd, ssim in report_sorted:
                f.write("{}, {:.6f}, {:.6f}\n".format(name, ssim, rmsd))

        _info("raycheck results written to {}".format(f.name))

    if os.path.isfile(args.regression_cutoffs):
        with open(args.regression_cutoffs, "r") as f:
            cutoffs_rmsd = {}
            cutoffs_ssim = {}
            for line in f:
                line = line.strip()
                if line.startswith("#"):
                    continue
                try:
                    name, ssim_lim, rmsd_lim = line.split("\t")

                    if name in cutoffs_rmsd:
                        _warn(
                            "Duplicate cutoffs detected for {}, using the last one".format(
                                name
                            )
                        )

                    cutoffs_rmsd[name] = float(rmsd_lim)
                    cutoffs_ssim[name] = float(ssim_lim)
                except:
                    _error(
                        'Failed to parse line "{}" in cutoffs file {}, ignoring'.format(
                            line, args.regression_cutoffs
                        )
                    )
                    continue

        for name, rmsd, ssim in report:
            if name not in cutoffs_rmsd:
                continue
            # Comparing the in-memory rounded numbers with rounded numbers
            # parsed from file can lead to fascinating conclusions like
            # 0.996457 > 0.996457 because floating points are a wonderful
            # construct with no problems whatsoever i love floating point i love
            # floating point i love floating point i love floating point i love
            #
            # Use comparisons with epsilon smaller than the precision of the
            # serialized format to dodge this problem.
            if rmsd >= cutoffs_rmsd[name] + 1e-7:
                _warn(
                    "{} RMSD {:.6f} is greater than cutoff {:.6f}".format(
                        name, rmsd, cutoffs_rmsd[name]
                    )
                )
            if ssim < cutoffs_ssim[name] - 1e-7:
                _warn(
                    "{} SSIM {:.6f} is lower than cutoff {:.6f}".format(
                        name, ssim, cutoffs_ssim[name]
                    )
                )
    else:
        _warn(
            "Cutoff file {} does not exist, not computing regression cutoffs".format(
                args.regression_cutoffs
            )
        )


def main():
    colorama.init()
    signal.signal(signal.SIGINT, signal.default_int_handler)

    parser = argparse.ArgumentParser(
        description="Compares your raytracer against the reference raytracer, creating diff files and reports",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--exec",
        metavar="RAY",
        help="Executable file for your ray tracer",
        default="build/bin/ray",
    )
    parser.add_argument(
        "--refbin",
        metavar="RAY.STD",
        help="Executable file for the reference ray tracer",
        default="ray-solution",
    )
    parser.add_argument(
        "--scenes",
        metavar="DIRECTORY",
        help="Directory that stores scene JSON files.",
        default="assets/scenes",
    )
    parser.add_argument(
        "--out", metavar="DIRECTORY", help="Output directory", default="raycheck.out"
    )
    """
    This option allows sharing the refcache
    """
    parser.add_argument(
        "--refcache",
        metavar="DIRECTORY",
        help="Directory of images created by the reference ray tracer.",
        default="",
    )
    parser.add_argument(
        "--json", metavar="JSON", help="JSON configuration file for testing", default=""
    )
    parser.add_argument(
        "--cubemap", metavar="FILE", help="One texture file for cubemapping", default=""
    )
    parser.add_argument(
        "--timelimit",
        metavar="SECONDS",
        help="Time limit in seconds. The default limit is used during grading. Files that time out are reported as having maximum error.",
        type=int,
        default=180,
    )
    parser.add_argument(
        "--nocompare", help="Only render the image, do not compare", action="store_true"
    )
    parser.add_argument(
        "--regression-cutoffs",
        help="CSV file which stores SSIM values that are considered regressions",
        default="cutoffs.csv",
    )
    parser.add_argument(
        "--i-understand-that-image-metrics-are-not-perfect",
        action="store_true",
        dest="suppress_memo",
    )
    args = parser.parse_args()

    raycheck(args)


if __name__ == "__main__":
    main()
