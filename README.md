# shadow_dexterous_hand
Meta package for the Shadow Dexterous Hand.

# Install
First grab a copy of `sr-build-tools-internal`: 
```
git clone git@github.com:shadow-robot/sr-build-tools-internal
```

then simply run (don't forget to substitute `yourgithublogin` and `yourgithubpassword`):

```
curl -L bit.ly/dev-machine | bash -s -- -w ~/workspace/dexterous_hand/base -r `pwd`/sr-build-tools-internal/data/dexterous_hand.rosinstall -l yourgithublogin -p yourgithubpassword
