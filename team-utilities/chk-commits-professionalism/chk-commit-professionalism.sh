#! /usr/bin/bash

for i in `cat badwords_en.txt`; do
  #git --no-pager grep -Ei '$i' $(git rev-list --all) 
  git --no-pager grep -Ei '$i' $(git for-each-ref --format='%(refname)' refs/remotes) 
  git --no-pager grep -Ei '$i' $(git for-each-ref --format='%(refname)' refs/heads) 

done
