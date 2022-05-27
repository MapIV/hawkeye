# Ortho Map Viewer
Updated (2022/05/11)

Publish topic of ortho image map for rivz.

## Usage

```
$ rosrun orthomap_viewer orthomap_viewer <ORTHOMAP_FILE> [ -t <TOPIC_NAME> ] [ -p <PCD_XYZI_FILE> ] [ -f <FRAME_NAME> ]
```

### Argument
* ORTHOMAP_FILE : The orthomap file.
  * ***The relative path to the ortho images must not be changed.***
  * ***The size of the whole pixels must be somewhat smaller than 1GB if you want to use rviz. The topic of the size larger than 1GB can not be shown in rviz.***
    * the resolution of the large file can be reduced by running ortho_image with a smaller scale in the yaml file. 
* TOPIC_NAME : The topic name of the map.
  * The default name is `ortho_map`.
* PCD_XYZI_FILE : PCD with XYZI points can be published simultaneously.
* FRAME_NAME : The target frame name of the map.

## Format of Orthomap File
Orthomap file is a text file containing information about the ortho images and their placement.
### Sample
```
orthomap 3
"map_"
"_i.tif"
true
0.125
-18874.938
-93250.062
6
5
4000
3000
18
true 1 255 true
0 1 1 1 1 0
0 0 0 1 1 0
0 1 0 0 1 1
1 1 1 1 1 1
0 1 1 1 0 0
```

This file means the 18 grayscale image files `map_00_i.tif` ... `map_17_i.tif` in the same directory as this file, which are 3000 pixels hight and 4000 pixels width, constitute the map as the following table.
The table shows the north on top, the coordinate of the left top pixel in the whole map is (-18874.938, -93250.062) in a coordinate system and the scale of the map is 0.125 m/pixel.

<table border="1">
    <tr>
      <td></td>
      <td>map_00_i.tif</td>
      <td>map_01_i.tif</td>
      <td>map_02_i.tif</td>
      <td>map_03_i.tif</td>
      <td></td>
    </tr>
    <tr>
      <td></td>
      <td></td>
      <td></td>
      <td>map_04_i.tif</td>
      <td>map_05_i.tif</td>
      <td></td>
    </tr>
    <tr>
      <td></td>
      <td>map_06_i.tif</td>
      <td></td>
      <td></td>
      <td>map_07_i.tif</td>
      <td>map_08_i.tif</td>
    </tr>
    <tr>
      <td>map_09_i.tif</td>
      <td>map_10_i.tif</td>
      <td>map_11_i.tif</td>
      <td>map_12_i.tif</td>
      <td>map_13_i.tif</td>
      <td>map_14_i.tif</td>
    </tr>
    <tr>
      <td></td>
      <td>map_15_i.tif</td>
      <td>map_16_i.tif</td>
      <td>map_17_i.tif</td>
      <td></td>
      <td></td>
    </tr>
 </table>

### Format
Line breaks in the samle above are meaningless; each element should be separated by one or more whitespace characters.

| Number | In the sample | Explanation |
| -----: | :-----------: | :---------- |
| 1 | `orthomap` | Format identifier. It must be `orthomap`. |
| 2 | `3` | Format version. This version is 3. Only the latest version is supported. |
| 3 | `"map_"` | Image filename prefix. It must be in double quotes and can be `""`. See the image names in the sample. |
| 4 | `"_i.tif"` | Image filename suffix. It must be in double quotes and can be `""`. See the image names in the sample.|
| 5 | `true` | Whether to fill the enumerating numbers in file names with zeros and make the number width the same length. It must be `true` or `false`. |
| 6 | `0.125` | Map scale. The images must be north at the top and northward and eastward scales must be the same. The unit is m/pixel. |
| 7 - 8 | `-18874.938 -93250.062` | X and y coordinates of the center of the left top pixel in the whole images in a coordinate system. 7th and 8th elements are same as the 5th element of the world files for the left-most image and the 6th element of those for the top-most image. The coordinate system used in the map cannot be determined from this format. |
| 9 - 10 | `6 5` | Width and height of the image table. |
| 11 - 12 | `4000 3000` | Width and height of each image file. |
| 13 | `18` | The number of image files. |
| 14 - 17 | `true 1 255 true` | About the pixel color. See [Color section](#color). If 14th element is `false`, then the 15th to 17th elements must be omitted. |
| 18 - | `0 1 1 1 1 0`<br>...<br>`0 1 1 1 0 0` | Whether an image exists in each position of the image table. The numbers are made into the table in row-major order and each element indicates existence and non-existence by 1 and 0. See the sample above. |

### Image Format
The images must consist of a single 8-bit channel.
They are expected to be PNG or TIFF, but other formats may also work.

#### Color
Since the color of each pixel must be represented in 256 steps, the color in the images may not same scale as the real color.
If the scales are different, set 14th element `true` and set 15th to 17th elements as following. Otherwise, set 14th element `false` and omit 15th to 17th elements.

* 15th element : The real color that `0`(`1` if 17th element is `true`) in the image shows.
* 16th element : The real color that `255` in the image shows.
* 17th element : Whether `0` is considered blank. If `0` is considered blank, set `true`. Otherwise, set `false`.

The following table is an example of 14th to 17th elements.

| 14th | 15th | 16th | 17th | Real color of `0` | Real color of `1` | Real color of `2` | Real color of `255` |
| :--: | :--: | :--: | :--: | :---------------: | :---------------: | :---------------: | :-----------------: |
| `false` | - | - | - | 0 | 1 | 2 | 255 |
| `true` | 0 | 255 | `false` | 0 | 1 | 2 | 255 |
| `true` | 0 | 1000 | `false` | 0 | 3.92 | 7.84 | 1000 |
| `true` | -100 | 100 | `false` | -100 | -99.2 | -98.4 | 100 |
| `true` | 1 | 255 | `true` | (emptiness) | 1 | 2 | 255 |
| `true` | 0 | 1000 | `true` | (emptiness) | 0 | 3.94 | 1000 |