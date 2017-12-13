import skimage.morphology
from scipy.misc import imread, imshow

def read_img():
	
	img = imread('cone_map.png')
	imshow(img)
	'''
	print img
	selem = skimage.morphology.disk(3)
	img = skimage.morphology.dilation(img, selem)
	#img = skimage.morphology.dilation(img, skimage.morphology.square(1))
	#img = skimage.morphology.dilation(img, skimage.morphology.square(1))
	#img = skimage.morphology.dilation(img, skimage.morphology.square(1))
	#img = skimage.morphology.dilation(img, skimage.morphology.square(1))

	imshow(img)
	'''
	return img
def load_map(map_img):
	cells = set()

	for i in range(len(map_img[0])):
	    for j in range(len(map_img)):
		#print "******MAP IMG*****", map_img[i][j]
		if map_img[i][j][0] == 255:
		    cells.add((j,i))

	return cells

if __name__ == '__main__':
	img = read_img()
	cells = load_map(img)
	print len(cells)
