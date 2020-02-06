#!/usr/bin/env python

import os
import tensorflow as tf
from PIL import Image, ImageDraw
from io import BytesIO
import argparse

if __name__ == "__main__":

        parser = argparse.ArgumentParser(description='')
        parser.add_argument(
                '-p', '--path', dest='path', action="store",
                help='the path of tfrecord files', default='', type=str)

        parser.add_argument(
                '-f', '--filename', dest='filename', action="store",
                help='the name of tfrecord file', default='*', type=str)

        parser.add_argument(
                '-o', '--output', dest='output', action="store",
                help='the output path of the cropped image', default='/tmp', type=str)

        parser.add_argument(
            '-s', '--show', dest='show', action="store",
                help='the show image', default=False, type=bool)

        args, unknowns = parser.parse_known_args()

        if args.filename == '*':
                args.show = False

        for file in os.listdir(args.path):
                base, ext = os.path.splitext(file)

                if ext == '.tfrecord':

                        if args.filename != '*' and args.filename != base:
                                        continue

                        print (file)
                        #filename =  args.path + '/' + args.filename + '.tfrecord'
                        filename =  args.path + '/' + file
                        raw_dataset = tf.data.TFRecordDataset(filename)

                        example = tf.train.Example()
                        for raw_record in raw_dataset.take(1):
                                example.ParseFromString(raw_record.numpy())
                        try:
                                width = example.features.feature['image/width'].int64_list.value[0]
                                height = example.features.feature['image/height'].int64_list.value[0]
                                raw_img = Image.open(BytesIO(example.features.feature['image/encoded'].bytes_list.value[0]))
                        except IndexError:
                                print('\033[33m' + file + 'has no correct image format (e.g. image/width, image/height and image/encoded)' + '\033[0m')

                        try:
                                xmin = width * example.features.feature['image/object/bbox/xmin'].float_list.value[0]
                                xmax = width * example.features.feature['image/object/bbox/xmax'].float_list.value[0]
                                ymin = height * example.features.feature['image/object/bbox/ymin'].float_list.value[0]
                                ymax = height * example.features.feature['image/object/bbox/ymax'].float_list.value[0]

                        except IndexError:
                                print('\033[33m' + file + 'has no bounding box information (e.g. image/object/bbox/xmin, xmax, ymin and ymax)' + '\033[0m')
                        draw = ImageDraw.Draw(raw_img)
                        draw.rectangle((xmin, ymin, xmax, ymax), fill=None, outline=(255, 255, 255))
                        if args.show:
                                raw_img.show()
                        cropped_img = raw_img.crop((xmin, ymin, xmax, ymax))
                        if args.show:
                                cropped_img.show()
                        cropped_img = cropped_img.convert("RGB")
                        cropped_img.save(args.output + '/' + base + '_cropped.jpg', quality=100)
                        #print(example)

        '''
        for k, v in example.features.feature.items():
                if k != 'image/encoded':
                        print(k)
                        print(v)
        '''

        '''

        def _parse_image_function(example_proto):
        return tf.io.parse_single_example(example_proto, image_feature_description)

        image_feature_description = {
        'image/encoded': tf.io.FixedLenFeature([], tf.string),
        }

        parsed_image_dataset = raw_dataset.map(_parse_image_function)
        print (parsed_image_dataset)

        for image_features in parsed_image_dataset:
        image_raw = image_features['image/encoded'].numpy()
        #print (image_raw)
        print(type(image_raw))
        print(len(image_raw))
        #im = np.array(image_raw)
        #print(image_raw.shape)
        #display.display(display.Image(data=image_raw))
        #pil_img = Image.open(BytesIO(image_raw))
        #pil_img.show()
        '''
