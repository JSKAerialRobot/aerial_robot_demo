#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'mbzirc2020_task1_tasks'

    ##### single detection model for bind model (drone AND ball)
    download_data(
        pkg_name=PKG,
        path='models/drone_detection_edgetpu_20200103_617.tflite',
        url='https://drive.google.com/uc?id=1xuZRpfeeGwe_A7GC6u-6RKtUm_CPF1Mk',
        md5='6aec8cf3ff7f9cdb5b15425b08cbf118',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='models/drone_detection_edgetpu_20200130_509.tflite',
        url='https://drive.google.com/uc?id=1DLCTH_MWnXKpQ9OMWZ4WRMeM0qQQ9JHT',
        md5='d0ea9489b52a2ef65cbc36aac559e722',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='models/drone_detection_edgetpu_20200204_611.tflite',
        url='https://drive.google.com/uc?id=1DAeD4b6Z0Kyusz_vw0pPuWAOKScdzMxV',
        md5='0f9b7cd7adf9530bd6ac4f4e6da61d72',
        extract=False,
    )

    ##### cacscaded detection model for separate model in bouding box (drone OR ball)
    download_data(
        pkg_name=PKG,
        path='models/cocompile_model_202001300509_202002030704.tar.gz',
        url='https://drive.google.com/uc?id=1zj3wJzbMTvjcpeuNjsww9wr3kShWgPuW',
        md5='07d7e9405477d6ef1256980b30a68ad9',
        extract=True,
    )


if __name__ == '__main__':
    main()

