#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'mbzirc2020_task1_vision'

    ## on-site testbed in Kashiwa Campus of The University of Tokyo, Chiba, Japan

    ### 20191229_kashiwa_DJI450_mavic2pro_1080HD
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20191229_kashiwa_DJI450_mavic2pro_1080HD.tar.gz',
        url='https://drive.google.com/uc?id=1LcZlm3sWepFaoT9Vlf3eRYAuVjWx_0eW',
        md5='691f82cc2594de5c6e20d5bc51040f41',
        extract=True,
    )

    ### 20200103_kashiwa_DJIF450_hydrus_rsd435_720HD
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200103_kashwia_DJI450_hydrus_rsd435_720HD.tar.gz',
        url='https://drive.google.com/uc?id=1IsSOzLfMc1iV2FBezmX0cXATRjrtUt7R',
        md5='7d611ba4c50683ac69cc27af2ab69d57',
        extract=True,
    )

    ### 20200121-outdoor-f450-net-hydrus-realsense-d435
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200121_kashwia_DJI450_hydrus_rsd435_720HD.tar.gz',
        url='https://drive.google.com/uc?id=1GrqoKGHVAoHFKc5yrqxZtaqqDBhYNylZ',
        md5='d0a4354cb0d3b748d3bff89d7fdae3ef',
        extract=True,
    )

    ### 20200122-outdoor-f450-net-hydrus-realsense-d435
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200122_kashwia_DJI450_hydrus_rsd435_720HD.tar.gz',
        url='https://drive.google.com/uc?id=1tjr7MP830zgSiBDhzS1L6aTnwLWqCUhl',
        md5='905e2e094ddc9e85b958e7d1b05fccc2',
        extract=True,
    )

    ### 20200124-outdoor-f450-net-hydrus-realsense-d435
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200124_kashwia_DJI450_hydrus_rsd435_720HD.tar.gz',
        url='https://drive.google.com/uc?id=1YIAihyEhEdJ79jwhgJsWCo1PVI2xsd4R',
        md5='f1359b60ddc3d1f785f2a4cd4e302341',
        extract=True,
    )

    ### 20200130-outdoor-f450-net-hydrus-realsense-d435
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200130_kashwia_DJI450_hydrus_rsd435_720HD.tar.gz',
        url='https://drive.google.com/uc?id=1O2f1yjs3GM4vjTvLZ_OFv44RC8H8t7zE',
        md5='87e2a5704ce9434479a44a3989255352',
        extract=True,
    )

    ### 20200204-outdoor-f450-net-hydrus-elp-fov60-1080HD
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200204-outdoor-f450-net-hydrus-elp-fov60-1080HD.tar.gz',
        url='https://drive.google.com/uc?id=1xFRl9W_q1Zziq2N-Z0WyOuRWg0Z-eVt7',
        md5='badfe6b7b37d183af53a152f183a4408',
        extract=True,
    )

    ## Trained model from above on-site testbed dataset, whcih will be used for transfer learning with followin real competition dataset
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/train_from_original_ssd_5000_batch64_augment_20191229_to_2020204_624.tar.gz',
        url='https://drive.google.com/uc?id=1iPDeKFAXwf7P1xq3OdGEfuYddDRz8nHK',
        md5='3f4211450b09ea4ea0a8c9769e8b1cba',
        extract=True,
    )


    ## real competition arena in ADENC, Abu Dhabi, UAE
    ### 20200220_MBZIRC_rehearsal_annotation_data
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200220_MBZIRC_rehearsal_annotation_data.tar.gz',
        url='https://drive.google.com/uc?id=1t3qOT1rA1grYCvBkllayMfzlySjdiJnT',
        md5='3ff090e0db6420ae70174d3d9de6ac66',
        extract=True,
    )

    ### 20200221_MBZIRC_rehearsal_annotation_data
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/mbzirc-rehearsal-annotation-20200221.tar.gz',
        url='https://drive.google.com/uc?id=1UF9MclkUWk13g7WWD7aWoaPjzfy-ueOP',
        md5='40137a4c51d7c688caa5a78e1dd8d2c7',
        extract=True,
    )

    ### 20200222_MBZIRC_rehearsal_annotation_data
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/mbzirc-rehearsal-annotation-20200222.tar.gz',
        url='https://drive.google.com/uc?id=1BtgWtJerdlRCZ6IR5ykGPz4Ruyzv44sa',
        md5='250a471914eca8669c9ce525ab1a3ff8',
        extract=True,
    )

    ### 20200223_MBZIRC_challenge_annotation_data
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/mbzirc-challenge-annotation-20200223.tar.gz',
        url='https://drive.google.com/uc?id=1lc7GKB_LqaAeeJjRM7XFSBMG3A0oQ9s0',
        md5='a237d87679855f70fb8457ff42a06689',
        extract=True,
    )



    ## cacscaded detection model for separate model in bouding box (drone OR ball)

    ### 20191229_kashiwa_DJI450_mavic2pro_1080HD cropped annotated data
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20191229_kashiwa_DJI450_mavic2pro_1080HD_cropped_annotated_data.tar.gz',
        url='https://drive.google.com/uc?id=1NOr4EFnBb3wlB9T9x0OulJM7wjtt-ahk',
        md5='5385d56f42f09000453235014f457960',
        extract=True,
    )

    ### 20200103_kashiwa_DJIF450_hydrus_rsd435_720HD corpped image
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200103_kashwia_DJI450_hydrus_rsd435_720HD_cropped_annotated_data.tar.gz',
        url='https://drive.google.com/uc?id=19m-jXm2w0OKcBa3IM4Rofrdxi3b5zsbh',
        md5='13a8372c096d1dc68836880500d0f114',
        extract=True,
    )


if __name__ == '__main__':
    main()

