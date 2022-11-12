#!/usr/bin/env python3
"""
This script will grab Tobii Glasses data off of an SD card reader and arrange
it into a more user-friendly structure. The data is selected and formatted by
a JSON file that the user inputs to the script.

Usage:
Input a JSON file that is stored in the location you want your data to wind up.
The script will create a folder with the same name as the JSON file.  Here's
how that file should be organized:

{
    "<project-id>": {
        "id": Name of folder to be placed in
        # if there are multiple recordings for this project
        "recordings": [("<recording-id>", "<trial_name>"), ...more recordings]
        # OR if there is only one recording to associate with the project
        "recordings": "<trial_name>"
        }
    },
    ...more projects
}

This will be arranged into this folder structure:
<Name of format JSON file>
    <id>
        <trial_name>.mp4
        <trial_name>.json
        ...
    ...
"""
import argparse
import json
import os
from glob import glob
from shutil import copyfile
import gzip
from tobii_data_process import process
import sys


def transfer_data(file, card, format_json, convert, verbose=True):
    """
    Transfers data from SD card
    """
    if verbose:
        print("Finding files...")

    if file is not None:
      card_base = os.path.join(file)
      print("Searching ", card_base)
    else:
      card_base = os.path.join('/Volumes/', card)
      if not os.path.exists(card_base):
          raise Exception('SD card not found!')

    with open(format_json) as f:
#       print("f: ", f)
#       print("f.read(): ", f.read())
        format_dict = json.load(f)
#       format_dict = json.loads(f.read())

#   print("format_dict: ", format_dict)

    transfers = []
#   targ_base = format_json.split('.json')[0]
    targ_base = ""

    for proj, info in format_dict.items():
#       print("proj, info: ", proj, info)
#       print("info['recordings']: ", info['recordings'])
#       print("len(info['recordings']): ", len(info['recordings']))
#       print("info['recordings'][0]: ", info['recordings'][0])
        if type(info['recordings']) == list:
#           for rec, trial in info['recordings']:
            for item in info['recordings']:
#               print("item: ", item)
#               print("type(item): ", type(item))
#               print("item.keys(): ", item.keys())
#               print("item.values(): ", item.values())
                for rec in item:
#                 print("rec, item[rec]: ", rec, item[rec])
                  trial = item[rec]
                print("rec, trial: ", rec, trial)
#               rec_path = os.path.join(card_base, 'projects', proj,
#                                       'recordings', rec)
                rec_path = os.path.join(card_base, 'projects', proj,
                                        'recordings', trial)
                print("rec_path: '%s'" % (rec_path))
                seg_paths = glob(os.path.join(rec_path, 'segments', '*'))
                print("seg_paths: ", seg_paths)
                if len(seg_paths) > 1:
                    for i in range(len(seg_paths)):
                        seg_path = seg_paths[i]
                        json_path = os.path.join(seg_path, 'livedata.json.gz')
                        vid_path = os.path.join(seg_path, 'fullstream.mp4')
                        transfers.append((json_path,
                                          os.path.join(targ_base,
                                                       info['id'],
#                                                      trial +
                                                       rec +
                                                       '_%i.json.gz' % i)))
                        transfers.append((vid_path,
                                          os.path.join(targ_base,
                                                       info['id'],
#                                                      trial +
                                                       rec +
                                                       '_%i.mp4' % i)))
                elif len(seg_paths) == 1:
                    seg_path = seg_paths[0]
                    json_path = os.path.join(seg_path, 'livedata.json.gz')
                    vid_path = os.path.join(seg_path, 'fullstream.mp4')
                    transfers.append((json_path,
                                      os.path.join(targ_base,
                                                   info['id'],
#                                                  trial +
                                                   rec +
                                                   '.json.gz')))
                    transfers.append((vid_path,
                                      os.path.join(targ_base,
                                                   info['id'],
#                                                  trial +
                                                   rec +
                                                   '.mp4')))
                else:
                    raise Exception('No segments found for id: ' % info['id'])
        else:
            rec_path = os.path.join(card_base, 'projects', proj, 'recordings',
                                    '*')
            rec_list = glob(rec_path)
            if len(rec_list) > 1:
                raise Exception('Multiple recordings found for id: %s, ' +
                                'but only one specified' % info['id'])
            elif len(rec_list) == 0:
                raise Exception('No recordings found for id: %s' % info['id'])
            else:
                rec_path = rec_list[0]
                seg_paths = glob(os.path.join(rec_path, 'segments', '*'))
                if len(seg_paths) > 1:
                    for i in range(len(seg_paths)):
                        seg_path = seg_paths[i]
                        json_path = os.path.join(seg_path, 'livedata.json.gz')
                        vid_path = os.path.join(seg_path, 'fullstream.mp4')
                        transfers.append((json_path,
                                          os.path.join(targ_base,
                                                       info['id'],
                                                       info['recordings'] +
                                                       '_%i.json.gz' % i)))
                        transfers.append((vid_path,
                                          os.path.join(targ_base,
                                                       info['id'],
                                                       info['recordings'] +
                                                       '_%i.mp4' % i)))
                elif len(seg_paths) == 1:
                    seg_path = seg_paths[0]
                    json_path = os.path.join(seg_path, 'livedata.json.gz')
                    vid_path = os.path.join(seg_path, 'fullstream.mp4')
                    transfers.append((json_path,
                                      os.path.join(targ_base,
                                                   info['id'],
                                                   info['recordings'] +
                                                   '.json.gz')))
                    transfers.append((vid_path,
                                      os.path.join(targ_base,
                                                   info['id'],
                                                   info['recordings'] +
                                                   '.mp4')))
                else:
                    raise Exception('No segments found for id: ' % info['id'])

    print("transfers: ", transfers)

    if verbose:
        print("Tranferring and unzipping data...")

    unzipped = move_and_unzip(transfers, verbose=verbose)

    print("unzipped: ", unzipped)

    if convert in (0, 1, 2):
        if verbose:
            print("Converting data into csv...")
            sys.stdout.write('  0.00%')
            i = 0.0

        for tobii_data in unzipped:
#           process(tobii_data, convert, verbose=False)
            process(tobii_data, convert, verbose=True)
            if verbose:
                i += 1
                sys.stdout.write('\r' + '%6.2f%%' % ((i / len(unzipped)) * 100))
                sys.stdout.flush()

    if verbose:
        print()
        print("Done!")


def move_and_unzip(transfers, verbose=True):
    unzipped = []

    if verbose:
        sys.stdout.write('  0.00%')
        i = 0.0

    for source, dest in transfers:
        dest_dir = os.path.dirname(dest)
        if not os.path.exists(dest_dir):
            os.makedirs(dest_dir)
        copyfile(source, dest)
        if dest.endswith('.gz'):
            unzip_dest = dest.split('.gz')[0]
            with gzip.open(dest) as infile:
                with open(unzip_dest, 'w+') as outfile:
                    for line in infile:
#                       outfile.write(line)
#                       outfile.write(str(line))
#                       print("line: ",line.decode("utf-8"))
                        outfile.write(line.decode("utf-8"))
            os.remove(dest)
            unzipped.append(unzip_dest)
        if verbose:
            i += 1
            sys.stdout.write('\r' + '%6.2f%%' % ((i / len(transfers)) * 100))
            sys.stdout.flush()
    print()
    return unzipped


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'format', help='JSON file that defines data to grab and file naming')
    parser.add_argument(
        '--file', help='Name of directory.', default=None)
    # 'NO NAME' is the default name of cards that come with the Tobii Glasses
    parser.add_argument(
        '--card', help='Name of SD card.', default='NO NAME')
    parser.add_argument(
        '--convert', help='Include this flag to convert tobii data into csv.' +
                          ' 0 for no cleaning. 1 for cleaning with linear' +
                          ' interpolation. 2 for cleaning with polynomial' +
                          ' interpolation.',
        default=-1)
    args = parser.parse_args()

    transfer_data(args.file, args.card, args.format, int(args.convert))
