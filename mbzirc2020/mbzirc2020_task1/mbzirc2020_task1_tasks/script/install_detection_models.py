#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'mbzirc2020_task1_tasks'

    download_data(
        pkg_name=PKG,
        path='models/drone_detection_edgetpu_617.tflite',
        url='https://drive.google.com/uc?id=1xuZRpfeeGwe_A7GC6u-6RKtUm_CPF1Mk',
        md5='6aec8cf3ff7f9cdb5b15425b08cbf118',
        extract=False,
    )

if __name__ == '__main__':
    main()

