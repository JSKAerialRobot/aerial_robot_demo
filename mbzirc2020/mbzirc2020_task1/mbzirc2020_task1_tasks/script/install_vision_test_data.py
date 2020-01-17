#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'mbzirc2020_task1_tasks'

    download_data(
        pkg_name=PKG,
        path='test/data/2020-01-03-kashiwa-hydrus-rsd435-task1-detection-rawdata2-1.bag',
        url='https://drive.google.com/uc?id=1jXnjkOYROV4V4UumxTgQ2-HfbOQwPmhl',
        md5='d283ffee47b8c875d63585ebfb7795c4',
        extract=False,
    )

if __name__ == '__main__':
    main()

