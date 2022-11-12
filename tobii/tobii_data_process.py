#!/usr/bin/env python
'''
    Process and sort sequential JSON data from Tobii eye tracker recordings
    into columnar format.  Also included option to clean and interpolate
    data for pupil diameter columns.
'''

import argparse
import json
import pandas as pd
import numpy as np


def num_lines(fname):
    with open(fname) as f:
        for i, l in enumerate(f, start=1):
            pass
    return float(i)


def read_data(json_fname, verbose=True):
    df = pd.DataFrame()
    pts_sync = {}
    vts_sync = {}
    pulse_sync = {}

    if verbose:
        print("Estimating size...")
    file_len = num_lines(json_fname)

    print('\njson_fname: ',json_fname)

    if verbose:
        print("Converting JSON...")
#   with open(json_fname) as f:
#   with open(json_fname,encoding='utf-8',errors='ignore') as f:
    with open(json_fname,encoding='utf-8-sig',errors='ignore') as f:
        i = 0
        for line in f:
#           line = str(line).strip("'<>() ").replace('\'', '\"')
#           line = line.decode('utf-8').replace('\0', '')
#           line = line.replace('\0', '')
#           print("line: ",line)
            entry = json.loads(line)
#           entry = json.loads(line,strict=False)
            i += 1
            if verbose:
                print(('[' + int((i / file_len) * 50) * '=' + int((1 - i / file_len) * 50) * '-' + ']'
                       ' %.1f %% Complete\r' % ((i / file_len) * 100)), end=' ')
            if entry['s'] != 0:
                continue
            elif 'dir' in list(entry.keys()):
                pulse_sync[entry['ts']] = entry['sig']
            elif 'pts' in list(entry.keys()):
                pts_sync[entry['ts']] = entry['pts']
                continue
            elif 'vts' in list(entry.keys()):
                vts_sync[entry['ts']] = entry['vts']
                continue
            if 'eye' in list(entry.keys()):
                which_eye = entry['eye'][:1]
                if 'pc' in list(entry.keys()):
                    df.loc[entry['ts'],
                           which_eye + '_pup_cent_x'] = entry['pc'][0]
                    df.loc[entry['ts'],
                           which_eye + '_pup_cent_y'] = entry['pc'][1]
                    df.loc[entry['ts'],
                           which_eye + '_pup_cent_z'] = entry['pc'][2]
                    df.loc[entry['ts'],
                           which_eye + '_pup_cent_val'] = entry['s']
                elif 'pd' in list(entry.keys()):
                    df.loc[entry['ts'],
                           which_eye + '_pup_diam'] = entry['pd']
                    df.loc[entry['ts'],
                           which_eye + '_pup_diam_val'] = entry['s']
                elif 'gd' in list(entry.keys()):
                    df.loc[entry['ts'],
                           which_eye + '_gaze_dir_x'] = entry['gd'][0]
                    df.loc[entry['ts'],
                           which_eye + '_gaze_dir_y'] = entry['gd'][1]
                    df.loc[entry['ts'],
                           which_eye + '_gaze_dir_z'] = entry['gd'][2]
                    df.loc[entry['ts'],
                           which_eye + '_gaze_dir_val'] = entry['s']
            else:
                if 'gp' in list(entry.keys()):
                    df.loc[entry['ts'], 'gaze_pos_x'] = entry['gp'][0]
                    df.loc[entry['ts'], 'gaze_pos_y'] = entry['gp'][1]
                    df.loc[entry['ts'],
                           'gaze_pos_val'] = entry['s']
                elif 'gp3' in list(entry.keys()):
                    df.loc[entry['ts'],
                           '3d_gaze_pos_x'] = entry['gp3'][0]
                    df.loc[entry['ts'],
                           '3d_gaze_pos_y'] = entry['gp3'][1]
                    df.loc[entry['ts'],
                           '3d_gaze_pos_z'] = entry['gp3'][2]
                    df.loc[entry['ts'],
                           '3d_gaze_pos_val'] = entry['s']

    # ATD: changed df.ix to df.loc
    df['pts_time'] = np.array(df.index)
    df.loc[df.index < min(sorted(pts_sync.keys())), 'pts_time'] = np.nan
    df['vts_time'] = np.array(df.index)
    df.loc[df.index < min(sorted(vts_sync.keys())), 'vts_time'] = np.nan

    for key in sorted(pts_sync.keys()):
        df.loc[df.index >= key, 'pts_time'] = np.array(
            df.index)[df.index >= key]
        df.loc[df.index >= key, 'pts_time'] = df.pts_time - key + pts_sync[key]

    for key in sorted(vts_sync.keys()):
        df.loc[df.index >= key, 'vts_time'] = np.array(
            df.index)[df.index >= key]
        df.loc[df.index >= key, 'vts_time'] = df.vts_time - key + vts_sync[key]
    if verbose:
        print()
    return df, pulse_sync

def window_diff(data, width):
    """
    Takes a series and calculates a diff between each value and the mean of
    values surrounding it (dictated by width) If this window extends past the
    data's indices, it will ignore those values.
    """
    diff = data.copy()
    for i in range(len(data)):
        if i < width:
            win_m = (np.nanmean(data[:i]) + np.nanmean(data[i+1:i+1+width]))/2
        else:
            win_m = (np.nanmean(data[i-width:i]) + np.nanmean(data[i+1:i+1+width]))/2
        diff[i] -= win_m
    return diff

def cleanseries(data, *args):
    if data.name != 'l_pup_diam' and data.name != 'r_pup_diam':
        return data
    interp_type = args[0]
    bad = (data == 0)

    print("data:\n",data)
    dd = window_diff(data, 10)
    sig = np.nanmedian(np.absolute(dd) / 0.67449)
    th = 5
    disc = np.absolute(dd) > th * sig

    to_remove = np.nonzero(bad | disc)[0]
    up_one = list(range(len(to_remove)))
    for i in range(len(to_remove)):
        up_one[i] = to_remove[i] + 1
    down_one = list(range(len(to_remove)))
    for i in range(len(to_remove)):
        down_one[i] = to_remove[i] - 1
    isolated = np.intersect1d(up_one, down_one)

    allbad = np.union1d(to_remove, isolated)

    newdat = data.copy()
    newdat[allbad] = np.nan

    goodinds = np.nonzero(np.invert(np.isnan(newdat)))[0]
    if len(goodinds) == 0:
        print("Not enough good data to clean. Aborting.")
        return data
    else:
        if interp_type == 1:
            return pd.Series.interpolate(newdat, method='linear')
        elif interp_type == 2:
            return pd.Series.interpolate(newdat, method='polynomial', order=3)


# adds 'seconds' column that converts tobii timestamps to seconds
def add_seconds(df):
    df = df.reset_index()
    df['seconds'] = (df['index'] - df['index'][0]) / 1000000.0
    df = df.set_index('index', drop=True)
    return df


def process(tobii_in, clean, verbose=True):

    print("tobii_in: ", tobii_in)
    print("Writing to ", tobii_in.rsplit('.',1)[0] + '.csv')

    print("Reading data...",)
    df, pulses = read_data(tobii_in, verbose=verbose)
    print("done.")

    if int(clean) in (1, 2):
        if verbose:
            print("Cleaning data...")
        df = df.reset_index()
        df = df.apply(cleanseries, args=[int(clean)])
        df = df.set_index('index', drop=True)
    df = add_seconds(df)

#   df.to_csv(tobii_in.split('.')[0] + '.csv')
    df.to_csv(tobii_in.rsplit('.',1)[0] + '.csv')
    if len(pulses) > 0:
#       with open(tobii_in.split('.')[0] + '_sync_pulses.json', 'w') as f:
        with open(tobii_in.rsplit('.',1)[0] + '_sync_pulses.json', 'w') as f:
            json.dump(pulses, f)
    if verbose:
        print("Done!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'tobii_in', help='Location of tobii JSON file to convert')
    parser.add_argument('--clean', default=0, help='Flag to clean pupil size data, 1 for linear interpolation, ' +
                                                   '2 for polynomial interpolation. Default is no cleaning.')
    args = parser.parse_args()

    process(args.tobii_in, args.clean)
