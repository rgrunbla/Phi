# Phi

## Build Instruction

```
mkdir build
cd build
cmake ../src/
make
```

## Updating ns3-zmq-messages

```
git fetch ns3-zmq-messages master
git subtree pull --prefix src/ns3-zmq-messages ns3-zmq-messages master --squash
```
