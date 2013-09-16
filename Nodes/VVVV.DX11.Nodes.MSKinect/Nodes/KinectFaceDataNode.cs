using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit.FaceTracking;
using SlimDX;
using VVVV.DX11.Nodes.MSKinect;
using VVVV.MSKinect.Lib;
using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VMath;

namespace MSKinect.Nodes
{
    [PluginInfo(Name = "FaceData", 
	            Category = "Kinect", 
	            Version = "Microsoft", 
	            Author = "vux", 
	            Tags = "DX11",
	            Help = "Returns detailed 2D and 3D data describing the tracked face")]
    public class KinectFaceFrameDataNode : IPluginEvaluate, IPluginConnections
    {
    	[Input("Kinect Runtime")]
        protected Pin<KinectRuntime> FInRuntime;
        
        [Input("Face", CheckIfChanged = true)]
        private Pin<FaceTrackFrame> FInFrame;
   
        [Input("World Space")]
        private ISpread<bool> FWorld;

        [Output("Success")]
        private ISpread<bool> FOutOK;

        [Output("Position")]
        private ISpread<Vector3> FOutPosition;

        [Output("Rotation")]
        private ISpread<Vector3> FOutRotation;

        [Output("Face Points")]
        private ISpread<ISpread<Vector3>> FOutPts;
        
        [Output("Face Points WorldSpace")]
        private ISpread<ISpread<Vector3>> FOutWorldPts;

        [Output("Face Normals")]
        private ISpread<ISpread<Vector3>> FOutNormals;
        
        [Output("Projected Face Points")]
        private ISpread<ISpread<Vector2>> FOutPPTs;

        [Output("Indices")]
        private ISpread<int> FOutIndices;

        private bool FInvalidateConnect = false;
        private bool first = true;
        private KinectRuntime runtime;

        public void Evaluate(int SpreadMax)
        {
        	if (this.FInvalidateConnect)
            {
                if (this.FInRuntime.PluginIO.IsConnected)
                {
                    //Cache runtime node
                    this.runtime = this.FInRuntime[0];
                }

                this.FInvalidateConnect = false;
            }
        	
            //Output static indices all the time
            if (this.first)
            {
                this.FOutIndices.AssignFrom(KinectRuntime.FACE_INDICES);
                this.first = false;
            }

            if (this.FInFrame.PluginIO.IsConnected)
            {
                if (this.FInFrame.IsChanged)
                {
                    this.FOutOK.SliceCount = FInFrame.SliceCount;
                    this.FOutPosition.SliceCount = FInFrame.SliceCount;
                    this.FOutRotation.SliceCount = FInFrame.SliceCount;
                    this.FOutPts.SliceCount = FInFrame.SliceCount;
                    this.FOutPPTs.SliceCount = FInFrame.SliceCount;

                    for (int cnt = 0; cnt < this.FInFrame.SliceCount; cnt++)
                    {
                        FaceTrackFrame frame = this.FInFrame[cnt];
                        this.FOutOK[cnt] = frame.TrackSuccessful;
                        this.FOutPosition[cnt] = new Vector3(frame.Translation.X, frame.Translation.Y, frame.Translation.Z);
                        this.FOutRotation[cnt] = new Vector3(frame.Rotation.X, frame.Rotation.Y, frame.Rotation.Z) * (float)VMath.DegToCyc;

                        EnumIndexableCollection<FeaturePoint, PointF> pp = frame.GetProjected3DShape();
                        EnumIndexableCollection<FeaturePoint, Vector3DF> p = frame.Get3DShape();

                        this.FOutPPTs[cnt].SliceCount = pp.Count;
                        this.FOutPts[cnt].SliceCount = p.Count;
                        this.FOutWorldPts[cnt].SliceCount = p.Count;
						this.FOutNormals[cnt].SliceCount = p.Count;

						//Compute smoothed normals
						Vector3[] norms = new Vector3[p.Count];
						int[] inds = KinectRuntime.FACE_INDICES;
						int tricount = inds.Length / 3;
						for (int j = 0; j < tricount; j++)
						{
							int i1 = inds[j * 3];
							int i2 = inds[j * 3 + 1];
							int i3 = inds[j * 3 + 2];

							Vector3 v1 = p[i1].SlimVector();
							Vector3 v2 = p[i2].SlimVector();
							Vector3 v3 = p[i3].SlimVector();

							Vector3 faceEdgeA = v2 - v1;
							Vector3 faceEdgeB = v1 - v3;
							Vector3 norm = Vector3.Cross(faceEdgeB, faceEdgeA);

							norms[i1] += norm; 
							norms[i2] += norm; 
							norms[i3] += norm;
						}
						
						for (int i = 0; i < pp.Count; i++)
						{
							this.FOutPPTs[cnt][i] = new Vector2(pp[i].X, pp[i].Y);
							var colorSpace = new Vector3(p[i].X, p[i].Y, p[i].Z);
							this.FOutPts[cnt][i] = colorSpace;
							
							var world = ConvertFromColorCameraSpaceToDepthCameraSpace(runtime, colorSpace);
							this.FOutWorldPts[cnt][i] = world;
							
							this.FOutNormals[cnt][i] = Vector3.Normalize(norms[i]);
						}

                        /*FaceTriangle[] d = frame.GetTriangles();
                        this.FOutIndices.SliceCount = d.Length * 3;
                        for (int i = 0; i < d.Length; i++)
                        {
                            this.FOutIndices[i * 3] = d[i].First;
                            this.FOutIndices[i * 3 + 1] = d[i].Second;
                            this.FOutIndices[i * 3 + 2] = d[i].Third;
                        }*/
                    }
                }
            }
            else
            {
                this.FOutPosition.SliceCount = 0;
                this.FOutPPTs.SliceCount = 0;
                this.FOutPts.SliceCount = 0;
                this.FOutRotation.SliceCount = 0;
                this.FOutOK.SliceCount = 0;
            }
        }

        //via http://nsmoly.wordpress.com/2012/05/21/face-tracking-sdk-in-kinect-for-windows-1-5/
		private Vector3 ConvertFromColorCameraSpaceToDepthCameraSpace(KinectRuntime runtime, Vector3 pPointInColorCameraSpace)
		{
			// Camera settings – these should be changed according to camera mode
			float depthImageWidth = 640.0f;
			float depthImageHeight = 480.0f;
			float depthCameraFocalLengthInPixels = runtime.Runtime.DepthStream.NominalFocalLengthInPixels;// NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
			float colorImageWidth = 640.0f;
			float colorImageHeight = 480.0f;
			float colorCameraFocalLengthInPixels = runtime.Runtime.ColorStream.NominalFocalLengthInPixels; //NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
			
			// Take a point in the depth camera space near the expected resulting point. Here we use the passed color camera space 3D point
			// We want to convert it from depth camera space back to color camera space to find the shift vector between spaces. Then
			// we will apply reverse of this vector to go back from the color camera space to the depth camera space
			var depthCameraSpace3DPoint = pPointInColorCameraSpace;
			
			// Project depth camera 3D point (0,0,1) to depth image
			var depthImage2DPoint = new DepthImagePoint();
			depthImage2DPoint.X = (int) (depthImageWidth * 0.5f + ( depthCameraSpace3DPoint.X / depthCameraSpace3DPoint.Z ) * depthCameraFocalLengthInPixels + 0.5);
			depthImage2DPoint.Y = (int) (depthImageHeight * 0.5f - ( depthCameraSpace3DPoint.Y / depthCameraSpace3DPoint.Z ) * depthCameraFocalLengthInPixels + 0.5);
			depthImage2DPoint.Depth = (int) (depthCameraSpace3DPoint.Z * 1000.0f);
				
			// Transform from the depth image space to the color image space
			var colorImage2DPoint = runtime.Runtime.CoordinateMapper.MapDepthPointToColorPoint(runtime.Runtime.DepthStream.Format, depthImage2DPoint, runtime.Runtime.ColorStream.Format);
//			NUI_IMAGE_VIEW_AREA viewArea = { NUI_IMAGE_DIGITAL_ZOOM_1X, 0, 0 };
//			HRESULT hr = NuiImageGetColorPixelCoordinatesFromDepthPixel(
//						NUI_IMAGE_RESOLUTION_640x480, &viewArea,
//						LONG(depthImage2DPoint.x + 0.5f), LONG(depthImage2DPoint.y+0.5f), USHORT(depthCameraSpace3DPoint.z*1000.0f) << NUI_IMAGE_PLAYER_INDEX_SHIFT,
//						&colorImage2DPoint.x, &colorImage2DPoint.y );
//			if(FAILED(hr))
//			{
//			ASSERT(false);
//			return hr;
//			}
			
			// Unproject in the color camera space
			var colorCameraSpace3DPoint = new Vector3();
			colorCameraSpace3DPoint.Z = depthCameraSpace3DPoint.Z;
			colorCameraSpace3DPoint.X = (( colorImage2DPoint.X - colorImageWidth*0.5f ) / colorCameraFocalLengthInPixels) * colorCameraSpace3DPoint.Z;
			colorCameraSpace3DPoint.Y = ((-colorImage2DPoint.Y + colorImageHeight*0.5f ) / colorCameraFocalLengthInPixels) * colorCameraSpace3DPoint.Z;
			
			// Compute the translation from the depth to color camera spaces
			var vTranslationFromColorToDepthCameraSpace = colorCameraSpace3DPoint - depthCameraSpace3DPoint;
			
			// Transform the original color camera 3D point to the depth camera space by using the inverse of the computed shift vector
			var pPointInDepthCameraSpace = pPointInColorCameraSpace - vTranslationFromColorToDepthCameraSpace;
			//XMStoreFloat3(pPointInDepthCameraSpace, v3DPointInKinectSkeletonSpace);
			
			return pPointInDepthCameraSpace;
		}
		
		public void ConnectPin(IPluginIO pin)
        {
            if (pin == this.FInRuntime.PluginIO)
            {
                this.FInvalidateConnect = true;
            }
        }

        public void DisconnectPin(IPluginIO pin)
        {
            if (pin == this.FInRuntime.PluginIO)
            {
                this.FInvalidateConnect = true;
            }
        }
    }
}
