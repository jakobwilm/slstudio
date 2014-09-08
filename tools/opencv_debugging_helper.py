# Put the following line into Qt Creator Debugger Preferences  "Additional Startup Commands"
# python exec(open("/home/jakw/Code/Repos/SLStudio/tools/opencv_debugging_helper.py").read())
# If put into "Debugging Helper Customization", the display formats are not properly registered (as of Qt Creator 3.0)
#

import numpy

def qform__cv__Mat():
	return "Normal,Displayed"
	#return "Normal"

def qdump__cv__Mat(d, value):
	ptrSize = d.ptrSize()

	dims = value['dims']
	refcount = value['refcount'].dereference()
	rows = value['rows']
	cols = value['cols']
	flags = value['flags']
	size = value['size']
	depth = flags & 7
	channels = 1 + (flags >> 3) & 63;
	if depth == 0:
		cv_type_name = 'CV_8U'
	elif depth == 1:
		cv_type_name = 'CV_8S'
	elif depth == 2:
		cv_type_name = 'CV_16U'
	elif depth == 3:
		cv_type_name = 'CV_16S'
	elif depth == 4:
		cv_type_name = 'CV_32S'
	elif depth == 5:
		cv_type_name = 'CV_32F'
	elif depth == 6:
		cv_type_name = 'CV_64F'
	#data = v['data'].cast(gdb.lookup_type("char").pointer())
	d.putValue('(%dx%d)' % (rows, cols))
	d.putNumChild(1)
	if d.isExpanded():
		with Children(d):
			#d.putIntItem('width', width)
			d.putIntItem('dims', dims)
			d.putIntItem('refcount', refcount)
			d.putIntItem('rows', rows)
			d.putIntItem('cols', cols)
			d.putIntItem('channels', channels)
			d.putIntItem('flags', flags)
			with SubItem(d, 'type'):
				d.putValue(cv_type_name)
				d.putType('int')
	format = d.currentItemFormat()
	if format == 1:
		d.putDisplay(StopDisplay)
	#elif format == 2:
		#if dims == 2:
			#img = cv2.cv.CreateImageHeader((cols,rows), depth, channels)
			#bytes = value['step'] * value['rows']
			#cv2.cv.SetData(img, d.readMemory(value['data'], bytes))
			#if channels == 1:
				#cv2.cv.CvtColor(img, img, cv2.cv.CV_GRAY2RGB)
			#d.putField("editformat", DisplayImageData)
			#d.put('editvalue="')
			#d.put('%08x%08x%08x%08x' % (cols, rows, byteSize, 13))
			#d.put(img.data)
			#d.put('",')

def qdump__cv__Size_(d, value):
	height = value.height
	width = value.width
	d.putNumChild(0)
	d.putValue('(%dx%d)' % (width, height))
	d.putType('cv::Size_')



