n= int(input("Enter the value of N"))

#Defining functions for adjacent nodes
def up(a,b):
    return(a,b+1)
def down(a,b):
    return(a,b-1)
def left(a,b):
    return(a-1,b)
def right(a,b):
    return(a+1,b)

#Main BFS logic function
def BFS(n):    
    #Creating a array for storing visited nodes
    marked=[]
    #Initialy marking every Node as False
    for i in range(0,n):
        rows=[]
        for j in range(0,n):
            rows.append(False)
        marked.append(rows)

    #Queue of nodes
    queue=[(0,0)]
    marked[0][0]=True #Start is marked

    #To store the order of nodes
    order=[]

    while(len(queue)>0):
        #Pop the first element of queue
        a,b=queue.pop(0)
        order.append((a,b))
            
        #Adjacent Nodes
        Neightbours=[up(a,b),down(a,b),left(a,b),right(a,b)]
        for x,y in Neightbours:
            if(0<=x<n and 0<=y<n and not marked[x][y]):
                marked[x][y]=True
                queue.append((x,y))
    
    print("Order of nodes visited   :\n")
    for node in order:
        print(f"{node}, ",end="")
    print()

#Calling the function
BFS(n)
