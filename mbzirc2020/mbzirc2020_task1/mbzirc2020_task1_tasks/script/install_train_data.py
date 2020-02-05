#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'mbzirc2020_task1_tasks'

    ### 20191229_kashiwa_DJI450_mavic2pro_1080HD
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20191229_kashiwa_DJI450_mavic2pro_1080HD.tar.gz',
        url='https://drive.google.com/uc?id=1LcZlm3sWepFaoT9Vlf3eRYAuVjWx_0eW',
        md5='691f82cc2594de5c6e20d5bc51040f41',
        extract=True,
    )

    ### 20191229_kashiwa_DJI450_mavic2pro_1080HD cropped annotated data
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20191229_kashiwa_DJI450_mavic2pro_1080HD_cropped_annotated_data.tar.gz',
        url='https://drive.google.com/uc?id=1NOr4EFnBb3wlB9T9x0OulJM7wjtt-ahk',
        md5='5385d56f42f09000453235014f457960',
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

    ### 20200103_kashiwa_DJIF450_hydrus_rsd435_720HD corpped image
    download_data(
        pkg_name=PKG,
        path='train/drone_and_ball/20200103_kashwia_DJI450_hydrus_rsd435_720HD_cropped_annotated_data.tar.gz',
        url='https://drive.google.com/uc?id=19m-jXm2w0OKcBa3IM4Rofrdxi3b5zsbh',
        md5='13a8372c096d1dc68836880500d0f114',
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

if __name__ == '__main__':
    main()

