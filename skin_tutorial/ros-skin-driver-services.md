# How to use the services of the skin driver without the API

Recommended ONLY for debugging and special setups. Please use the TUM ICS skin
API classes to access the skin. It is much easier to use and provides a lot of 
extra functionalities for instance access to patch configuration files.


## Find the services of the skin driver

There might be several in different namespaces.


```bash
rosservice list
```

Example output for skin driver without a namespace:

```
/createSkinCellDataPub
/enableSkinCellDataPub
/getSkinCellIds
/getSkinCellNeighbors
/setSkinCellLedColor
```

## Get a list of skin cell IDs of all connected skin cells

```bash
rosservice call /getSkinCellIds
```

Example output for skin patch with 7 skin cells:
```
cellId: [1, 2, 7, 3, 5, 6, 4]
ok: True
```



## Set the color of skin cell 1 to red

```bash
rosservice call /setSkinCellLedColor "color:
- cellId: 1
  r: 255
  g: 0
  b: 0" 
```

Output:
```
ok: True
```


## Set the color of all skin cells to green


```bash
rosservice call /setSkinCellLedColor "color:
- cellId: 16383
  r: 0
  g: 255
  b: 0"
```

Output:
```
ok: True
```


## Set the color of several skin cells
```bash
rosservice call /setSkinCellLedColor "color:
- cellId: 1
  r: 0
  g: 0
  b: 255
- cellId: 2
  r: 255
  g: 0
  b: 0"
```

Output:
```
ok: True
```

## Get the sensor data of skin cell 1

### 1) Create a topic to publish the data of skin cell 1 in topic 'skinTest1'

```bash
rosservice call /createSkinCellDataPub "topicName: 'skinTest1'
cellId:
- 1"
```

Output:
```
ok: True
```

Check if the topic has been created:
```bash
rostopic list
```

The output should contain one line with:
```
/skinTest1
```

### 2) Enable the publisher to publish data in the topic 'skinTest1'

```bash
rosservice call /enableSkinCellDataPub "topicName: 'skinTest1'
enable: true"
```

Output:
```
ok: True
```

### 3) Echo the topic

```bash
rostopic echo /skinTest1
```

The program should now continuousely output the sensor measurements of the skin cell.
The output could look like:
```
data: 
  - 
    cellId: 1
    ts: 64180003845
    prox: [0.0]
    force: [0.0009765625, 0.0009765625, 0.0]
    acc: [0.01953125, 0.03515625, 1.01171875]
    temp: [31.5, 0.0]
```


## Get the sensor data of several skin cells

NOTE: the array CAN contain several skin data items, but NOT necessarily, see
output of step 3). 


### 1) Create a topic to publish the data of skin cell 1 in topic 'skinTest2'

```bash
rosservice call /createSkinCellDataPub "topicName: 'skinTest2'
cellId:
- 1
- 2
- 3
- 4"
```

Output:
```
ok: True
```

### 2) Enable the publisher to publish data in the topic 'skinTest2'

```bash
rosservice call /enableSkinCellDataPub "topicName: 'skinTest2'
enable: true"
```

Output:
```
ok: True
```

### 3) Echo the topic

```bash
rostopic echo /skinTest2
```

The output could look like:
```
---
data: 
  - 
    cellId: 1
    ts: 240987230050
    prox: [4.57763671875e-05]
    force: [0.0, 0.0009765625, 0.0]
    acc: [0.0390625, 0.03515625, 1.01171875]
    temp: [32.0, 0.0]
---
data: 
  - 
    cellId: 4
    ts: 240989432087
    prox: [0.00152587890625]
    force: [0.0009765625, 0.0, 0.0]
    acc: [0.03125, -0.0390625, 1.0234375]
    temp: [30.5, 0.0]
  - 
    cellId: 2
    ts: 240989455123
    prox: [0.0]
    force: [0.0009765625, 0.0009765625, 0.0]
    acc: [-0.0, 0.06640625, 1.07421875]
    temp: [29.0, 0.0]
---
data: 
  - 
    cellId: 3
    ts: 240989947492
    prox: [0.006866455078125]
    force: [0.0, 0.0009765625, 0.001953125]
    acc: [-0.0546875, 0.0234375, 0.99609375]
    temp: [29.5, 0.0]
---
```

