# Things to work on (ML)

1 - Collect or generate aerial top-down images of tents and mannequins.
    Include partial random occlusions (shadows, branches, bushes), randomly rotated and randomly positioned mannequins, different scale, shaded, too bright and too dark situations. use photoshop or generate them or use 3d editing tools to recreate situations

2 - Train the hard negatives(very important)
    images without tents and mannequins, see what it detects incorrectly and include them as negatives (rocks, boulders, cars, animals, etc)

3 - Mess with SAHI. See if its better when cropping at 128x128 or 256x256 or 512x512 etc. Mess with the overal ratio (0.25, 0.30, etc)

4 - Try yolo11m as well. 

5 - Finetune the model

6 - Tune the confidence

# What we found so far

# Object Size
In our case(from 150ft above the ground), 
    - Our camera's horizontal FOV is around 81 degrees
    - Ground width ≈ 78.1 m, so pixels per meter ≈ 24.6 px/m.
    - 3.0 m square tent side: ~74 px
    - 2.4 m tent side: ~59 px
    - Mannequin length (≈1.8 m): ~44 px
    - Mannequin width (≈0.5 m from above): ~12 px

# Classifier
    MobileNet is not going to improve our workflow, in fact it slows it down. 
    Whatever it can do, Yolo11s can do better, more so, it already has a built in classifier and it will handle our detection way bettter