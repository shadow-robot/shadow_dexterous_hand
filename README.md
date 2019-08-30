# shadow_dexterous_hand
Meta package for the Shadow Dexterous Hand.

(https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoieG45cHNBK3JGMU5kcG1jQUdpMjUrTC96Q1lVWWVFZGhiYXE4VVFWSHNzai9MZ2lXRjZMV2FMbmp1bmxuQkhDa0hWcC80NWhwOEpISVpSVXdpZWdIN3l3PSIsIml2UGFyYW1ldGVyU3BlYyI6IkVWaUt1WUR0ZXkxR05MMTIiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=F%23SRC-3404_AWS_Docker_Image_Release)

# Install
First grab a copy of `sr-build-tools`: 
```
git clone git@github.com:shadow-robot/sr-build-tools
```

then simply run

```
curl -L bit.ly/dev-machine | bash -s -- -w ~/workspace/dexterous_hand/base -r `pwd`/sr-build-tools/data/dexterous_hand.rosinstall
