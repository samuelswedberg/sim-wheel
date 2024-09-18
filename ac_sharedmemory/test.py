from pyaccsharedmemory import accSharedMemory

asm = accSharedMemory()

while(1):
    sm = asm.read_shared_memory()
    if (sm is not None):
        print(f"Gear: {sm.Physics.rpm}")
    
