import numpy as np 
import open3d
import time
    

class Sphere():
    def __init__(self,freq):
        # self.create_Octahedral()
        # self.display()
        # for i in range(0,freq):
            # start = time.time()
            # self.subdivide()
            # print("subdivision",i,time.time()-start)
        # self.project()
        # self.display()
        self.from_one_triangle(freq)

        
    def from_one_triangle(self,freq):
        self.create_triangle()
        # self.display()
        for i in range(0,freq):
            start = time.time()
            self.subdivide()
            print("subdivision",i,time.time()-start)
        self.project()

        for j in range(0,3):
            self.vertices2 = np.array(self.vertices.copy())
            if(j==0):
                self.vertices2[:,0] = self.vertices2[:,0] * -1.0
            elif(j==1):
                self.vertices2[:,:2] = self.vertices2[:,:2] * -1.0
            else:
                self.vertices2[:,1] = self.vertices2[:,1] * -1.0

            self.vertices  = self.vertices + list(self.vertices2)

            self.faces2 = self.faces.copy()
            self.faces2 = np.array(self.faces2) + np.array(self.vertices2).shape[0]
            self.faces = self.faces + list(self.faces2)

        mirror = np.array(self.vertices.copy())
        rota = np.array([[np.cos(np.pi/2),0,np.sin(np.pi/2)],
                         [0,1.0,0.0],
                         [-np.sin(np.pi/2),0,np.cos(np.pi/2)]])
        mirror = np.dot(mirror,rota)
        self.vertices  = self.vertices + list(mirror)

        self.faces2 = self.faces.copy()
        self.faces2 = np.array(self.faces2) + np.array(mirror).shape[0]
        self.faces = self.faces + list(self.faces2)

    def display(self):
        edges    = []
        for f in self.faces:
            edges.append([f[0],f[1]])
            edges.append([f[1],f[2]])
            edges.append([f[2],f[0]])

        line_set = open3d.geometry.LineSet()
        line_set.points=open3d.utility.Vector3dVector(np.array(self.vertices))
        line_set.lines=open3d.utility.Vector2iVector(edges)
        colors = np.ones((np.array(edges).shape[0],3))
        colors = colors * [0.9,0.6,0.693]
        line_set.colors = open3d.utility.Vector3dVector(colors)

        pcl = open3d.geometry.PointCloud()
        pcl.points = open3d.utility.Vector3dVector(np.array(self.vertices))

        open3d.visualization.draw_geometries([line_set])

    def create_triangle(self):
        X = 1.0
        
        self.vertices = [[0.0, X, 0.0], 
                        [X, 0.0,  -X],
                        [X, 0.0,   X]
                        ]
        self.faces = [[0, 1, 2]]

    def create_Octahedral(self):
        X = 1.0
        
        self.vertices = [[0.0, X, 0.0], 
                        [X, 0.0,  -X],
                        [X, 0.0,   X], 
                        [-X, 0.0,  X],
                        [-X, 0.0, -X], 
                        [0.0, -X, 0.0]
                        ]
        self.faces = [[0, 1, 2], 
                        [0, 2, 3],
                        [0, 3, 4],
                        [0, 4, 1],
                        [5, 2, 1], 
                        [2, 5 ,3], 
                        [3, 5 ,4],
                        [4, 5 ,1]
                        ]
    def middle(self,p1,p2):
        return list((np.array(p1)+np.array(p2))/2)
    def check_if_notexist(self,truc,p):
        return not(p in truc)
    def get_idx(self,truc,p):
        return truc.index(p)
       
   
    def project(self):
        vert = np.array(self.vertices)
        ptsnew = np.hstack((vert, np.zeros(vert.shape)))
        xy = vert[:,0]**2 + vert[:,1]**2
        ptsnew[:,3] = np.sqrt(xy + vert[:,2]**2)
        ptsnew[:,4] = np.arctan2(np.sqrt(xy), vert[:,2]) # for elevation angle defined from Z-axis down
        #ptsnew[:,4] = np.arctan2(xyz[:,2], np.sqrt(xy)) # for elevation angle defined from XY-plane up
        ptsnew[:,5] = np.arctan2(vert[:,1], vert[:,0])
        pts = ptsnew[:,4:]
        theta = pts[:,1]
        phi   = pts[:,0]

        x =  np.sin(phi) * np.cos(theta)
        y =  np.sin(phi) * np.sin(theta)
        z =  np.cos(phi)
        self.vertices = list(np.concatenate( [ np.expand_dims(x,1),np.expand_dims(y,1),np.expand_dims(z,1) ],1))


            

    def subdivide(self):
        new_faces  = []
        for f in self.faces:
            p1 = self.vertices[f[0]]
            p2 = self.vertices[f[1]]
            p3 = self.vertices[f[2]]

            p1_b = self.middle(p1,p2)
            p2_b = self.middle(p2,p3)
            p3_b = self.middle(p3,p1)

            idxs = []
            for p in [p1_b, p2_b,p3_b]:
                if(self.check_if_notexist(self.vertices,p)):
                    self.vertices.append(p)
                    idxs.append(len(self.vertices)-1)
                else:
                    ind = self.get_idx(self.vertices,p)
                    idxs.append(ind)
            new_faces.append([f[0],idxs[0],idxs[2]])
            new_faces.append([idxs[0],f[1],idxs[1]])
            new_faces.append([idxs[2],idxs[1],f[2]])
            new_faces.append([idxs[0],idxs[1],idxs[2]])
        self.faces = new_faces
        

            



if __name__ == "__main__":
  sphere = Sphere(4)
  