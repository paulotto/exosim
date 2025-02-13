# Bone Meshes
- Unit: [m]
- The reference coordinate system for all meshes is the center of mass of the skull mesh
- Mandibular condyles and fossae are additionally upsampled and smoothed 
(MeshLab: Butterfly subdivision + Laplacian smooth)
- Reference frame and unit of the vectors can be changed with functions of the python script 
[utils_obj_files.py](../../../scripts/utils_obj_files.py)

## Source
The initial mesh files were obtained from the *larynx* demo of the [*ArtiSynth*](https://www.artisynth.org/Software/ModelsDownload) framework. 


Copyright (c) 2014, ArtiSynth, A. G. Hannam

All rights reserved.

THIS SOFTWARE AND DATA IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
