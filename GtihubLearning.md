###  Whats GIT?
    its a file that either lives in the home directory(global space) or in .git folder
### this is how git stores your files 

commit points to the tree snapshot
tree is the folder/file name + link(pointer) to the blob
blob is binary long object which is the raw chunk of data

How everything is chained together 
    commit → tree → blob
                      ↳ file content

each of these are objects(commit, tree, blob)

