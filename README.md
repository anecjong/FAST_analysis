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

<img width="100%" src="https://user-images.githubusercontent.com/99626367/176428653-4b27c591-5ba9-4cdc-a887-8aeed04069d5.png"/>

response 20~30 0 10% of matching pts

<img width="100%" src="https://user-images.githubusercontent.com/99626367/176428664-8bac77ee-8f79-4270-ab94-d49560a2b836.png"/>

response 30~40 - 10% of matching pts.

<img width="100%" src="https://user-images.githubusercontent.com/99626367/176428754-7faf6441-b9b5-4d9c-bff5-231cbcb99bc0.png"/>

threshold: 30, matchings with small hamming distance

<img width="100%" src="https://user-images.githubusercontent.com/99626367/176428763-5b10eee8-2e18-4b05-98eb-d8557c6e9396.png"/>

threshold: 30, matchings with large hamming distance