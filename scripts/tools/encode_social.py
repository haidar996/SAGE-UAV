#!/usr/bin/env python3
"""Re-encode a video to the format social platforms recommend (LinkedIn, YouTube, Instagram):
MP4 container, H.264 (High, level 4.0), yuv420p, constant 30 fps, AAC stereo audio (a silent track is added because
some uploaders expect one), MP4 index at the start (faststart).
usage: python3 scripts/tools/encode_social.py in.mp4 out.mp4 [crf=20]
Needs ffmpeg: it uses the copy bundled with the 'imageio-ffmpeg' pip package (pip install imageio-ffmpeg) or a system ffmpeg."""
import shutil
import subprocess
import sys


def ffmpeg_exe():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        exe = shutil.which('ffmpeg')
        if not exe:
            sys.exit('ffmpeg not found: pip install imageio-ffmpeg')
        return exe


def main(src, dst, crf='20'):
    cmd = [ffmpeg_exe(), '-y', '-hide_banner', '-loglevel', 'error',
           '-i', src, '-f', 'lavfi', '-i', 'anullsrc=channel_layout=stereo:sample_rate=44100',
           '-map', '0:v:0', '-map', '1:a:0', '-shortest',
           '-c:v', 'libx264', '-profile:v', 'high', '-level', '4.0', '-pix_fmt', 'yuv420p', '-preset', 'fast', '-crf', str(crf),
           '-vf', 'fps=30', '-r', '30', '-g', '60', '-c:a', 'aac', '-b:a', '128k', '-ar', '44100', '-ac', '2',
           '-movflags', '+faststart', dst]
    subprocess.run(cmd, check=True)
    print('encoded', dst)


if __name__ == '__main__':
    main(*sys.argv[1:4])
