# shadow_dexterous_hand
Meta package for the Shadow Dexterous Hand.

| Build server  | Status |
|---------------|--------|
| AWS | ![Build Status](https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiSkhPRGh2SzlBSVNMN1BPcjVMQlFsd3phYXVRSFVUTHV6UjVQUHdrMzc5ZW02WUZNNHNRSjJpbVduZnhZbTFXbjU4aHNJZElsQnJGdlYvSHV1WlRxVUhVPSIsIml2UGFyYW1ldGVyU3BlYyI6IkFyVmxzQ2NGSW9HcUZjLzciLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=F#SRC-3404_AWS_Docker_Image_Release) |

# Install
First grab a copy of `sr-build-tools`: 
```
git clone git@github.com:shadow-robot/sr-build-tools
```

then simply run

```
curl -L bit.ly/dev-machine | bash -s -- -w ~/workspace/dexterous_hand/base -r `pwd`/sr-build-tools/data/dexterous_hand.rosinstall
