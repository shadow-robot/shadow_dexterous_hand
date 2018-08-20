# shadow_dexterous_hand
Meta package for the Shadow Dexterous Hand.

# Install
First grab a copy of `sr-build-tools`: 
```
git clone git@github.com:shadow-robot/sr-build-tools
```

then simply run

```
curl -L bit.ly/dev-machine | bash -s -- -w ~/workspace/dexterous_hand/base -r `pwd`/sr-build-tools/data/dexterous_hand.rosinstall
