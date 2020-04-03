---
layout: post
title: My first post
---

Hi my name is Søren 

Some mathjax: $a^2 + b^2 = c^2$

Some more:

$$Jdq = dX$$

Some code:
~~~matlab
classdef (Abstract) Obstacle
    
    properties (Abstract)
        center
        colMesh %Collision Mesh
    end
    
    methods (Abstract)
        nearestPoint(obj)
    end
    
end
~~~

An image:

![SHP logo](/images/shp.png)