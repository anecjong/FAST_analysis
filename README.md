<div align="center">
  <h1>FAST Analysis</h1>
</div>

## FAST keypoints + r-BRIEF analysis with changing threshold
```
./FAST_ANALYSIS 0
./FAST_ANALYSIS 1
```
In build directory, type above.
0: kitti_far, 1: kitti_close images are used.

```
====thresh 40 matching====
1st NoF:           1201
1st response:      78.8984
1st time:          473.7
2nd NoF:           1213
2nd response:      78.4328
2nd time:          434.1
Accuracy:          0.0490094
matching accuracy: 89 %
Matching cnt:      959
Desc time:         3386
Matching time:     5280.6
Homography time:   1717.07
```
Number of features, response(how good corner is), average time for 5 calculations, matching count, matching accuracy(hamming distance below 64), matching counts are printed in terminal.

Matching info is also given by images.  
- match_imgs_threshold
threshold change and in for hamming distance l1 below 64 and out for over 64.

- match_imgs_response  
20 pts for each response and 10% of responses


![picture 8](../images/2402bdf223327541310bb98950533b65993b0c2952113268252294bf84e1d680.png)  
response 20~30 0 10% of matching pts

![picture 9](../images/f8d8645632d93d27ab3a688579a35b999c60469849f6f92e1146a6d4927bc718.png)  
response 30~40 - 10% of matching pts.

![picture 6](../images/6212b8f6e994f38f0c7d7fe73c16726e5ac88341d7299204a7034285dd946696.png)  
threshold: 30, matchings with small hamming distance

 ![picture 7](../images/94269f3a728646a0f6b547f7e1171c23c1f23c2a9ad1865fc62bf7cb843f3405.png)  
threshold: 30, matchings with large hamming distance