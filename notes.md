# Computer Vision

Setup:
```
> git remote add upstream https://github.com/duckietown/lx-computer-vision 
> git remote -v
> git pull upstream ente
```

Testing
```
Start duckie in matrix
> dts duckiebot virtual start bobbyvirt
> dts fleet discover
> dts code start_matrix

> dts code build -R bobbyvirt
> dts code workbench -m -R bobbyvirt 
> code: quackquack
> dts code vnc -R bobbyvirt

> docker -H bobbyvirt

TBD
> dts duckiebot keyboard_control bobbyvirt

```


## Lane Following

- [lane following demo](https://docs.duckietown.com/ente/duckietown-manual/40-demonstrations/lane-following-lf.html)


```
> dts duckiebot virtual start bobbyvirt
> dts matrix run --standalone --embedded --map sandbox
> dts matrix attach bobbyvirt map_0/vehicle_0
> dts duckiebot demo --demo_name lane-following --duckiebot_name bobbyvirt --debug

> dts duckiebot keyboard_control bobbyvirt
> dts duckiebot image_viewer bobbyvirt


TBD
> dts gui bobbyvirt
> rqt_image_view


```




```

```
