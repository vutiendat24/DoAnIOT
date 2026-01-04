
import type React from "react"
import { useState, useEffect, useRef } from "react"
import {
  Camera,
  AlertCircle,
  Users,
  Activity,
  Settings,
  Bell,
  Upload,
  Trash2,
  Clock,
  MapPin,
  Eye,
  Download,
  X,
  BellOff,
  Video,
  Maximize2,
  CheckCircle,
  DoorOpen,
  Loader2,
} from "lucide-react"

const API_BASE_URL = "http://localhost:8000/api"
const WS_URL = "ws://localhost:8000/ws/detections"
const MOCK_TOKEN = "test_token"

type Detection = {
  id: string
  timestamp: string
  image_url?: string
  alert: boolean
  detections: Array<{
    face_id: string
    confidence: number
    alert: boolean
  }>
  event_id?: string
}

type Notification = Detection & {
  read: boolean
}

const App = () => {
  const [activeTab, setActiveTab] = useState("dashboard")
  const [events, setEvents] = useState<Detection[]>([])
  const [liveDetections, setLiveDetections] = useState<Detection[]>([])
  const [stats, setStats] = useState({ total: 0, alerts: 0, known: 0, today: 0 })
  const [loading, setLoading] = useState(false)
  const [rois, setRois] = useState([])
  const [alertsOnly, setAlertsOnly] = useState(false)
  const [notifications, setNotifications] = useState<Notification[]>([])
  const [notificationsEnabled, setNotificationsEnabled] = useState(true)
  const [selectedImage, setSelectedImage] = useState<string | null>(null)
  const [liveStream, setLiveStream] = useState<string | null>(null)
  const [isStreamActive, setIsStreamActive] = useState(false)
  const [doorLoading, setDoorLoading] = useState(false)

  const wsRef = useRef<WebSocket | null>(null)
  const fileInputRef = useRef<HTMLInputElement>(null)
  const audioRef = useRef<HTMLAudioElement>(null)

  const apiCall = async (endpoint: string, options: RequestInit = {}) => {
    const url = `${API_BASE_URL}${endpoint}`
    const headers = {
      Authorization: `Bearer ${MOCK_TOKEN}`,
      ...options.headers,
    }

    const response = await fetch(url, { ...options, headers })

    if (!response.ok) {
      const error = await response.json().catch(() => ({ detail: "Request failed" }))
      throw new Error(error.detail || `HTTP ${response.status}`)
    }

    return response.json()
  }

  const playAlertSound = () => {
    if (audioRef.current && notificationsEnabled) {
      audioRef.current.play().catch(() => { })
    }
  }

  const showNotification = (detection: Detection) => {
    if (!notificationsEnabled) return

    const notification: Notification = {
      ...(detection as Detection),
      id: "10",
      read: false,
    }

    setNotifications((prev) => [notification, ...prev].slice(0, 10))
    playAlertSound()

    setTimeout(() => {
      setNotifications((prev) => prev.filter((n) => n.id !== notification.id))
    }, 8000)
  }

  const connectWebSocket = () => {
    try {
      const ws = new WebSocket(WS_URL)

      ws.onopen = () => {
        setIsStreamActive(true)
      }

      ws.onmessage = (event) => {
        const data = JSON.parse(event.data)
        if (data.type === "detection") {
          setLiveDetections((prev) => [data.data, ...prev.slice(0, 4)])
          setLiveStream(data.data.image_url)
          showNotification(data.data)
          fetchEvents()
        }
      }

      ws.onerror = () => {
        setIsStreamActive(false)
      }

      ws.onclose = () => {
        setIsStreamActive(false)
        setTimeout(connectWebSocket, 3000)
      }

      wsRef.current = ws
    } catch (error) {
      setIsStreamActive(false)
    }
  }

  useEffect(() => {
    connectWebSocket()
    return () => {
      if (wsRef.current) {
        wsRef.current.close()
      }
    }
  }, [])

  const fetchEvents = async () => {
    try {
      setLoading(true)
      const data = await apiCall(`/events?limit=50&alert_only=${alertsOnly}`)
      setEvents(data)
      if(alertsOnly === true){
        setEvents(events.filter(event => event.alert === true))
      }
      const total = data.length
      const alerts = data.filter((e: Detection) => e.alert).length
      const today = data.filter((e: Detection) => {
        const eventDate = new Date(e.timestamp)
        const now = new Date()
        return eventDate.toDateString() === now.toDateString()
      }).length

      setStats({ total, alerts, known: total - alerts, today })
    } catch (error) {
      console.error("Failed to fetch events:", error)
    } finally {
      setLoading(false)
    }
  }

  const fetchROIs = async () => {
    try {
      const data = await apiCall("/roi")
      setRois(data)
    } catch (error) {
      console.error("Failed to fetch ROIs:", error)
    }
  }

  // const handleImageUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
  //   const file = event.target.files?.[0]
  //   if (!file) return

  //   const formData = new FormData()
  //   formData.append("file", file)

  //   try {
  //     setLoading(true)
  //     const data = await apiCall("/detect", {
  //       method: "POST",
  //       body: formData,
  //     })

  //     alert(`Detection complete! Found ${data.detections.length} person(s)`)
  //     fetchEvents()
  //   } catch (error) {
  //     console.error("Detection failed:", error)
  //     alert("Detection failed: " + (error as Error).message)
  //   } finally {
  //     setLoading(false)
  //   }
  // }
  const handleImageUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    console.log("Uploading file:", {
      name: file.name,
      type: file.type,
      size: file.size,
    });

    const formData = new FormData();
    formData.append("file", file);

    try {
      setLoading(true);

      // Gọi API detect mà KHÔNG set Content-Type
      const data = await apiCall("/detect", {
        method: "POST",
        body: formData,
        headers: { Authorization: `Bearer ${MOCK_TOKEN}` },
      });

      console.log("Server response:", data);

      if (data.detections?.length >= 0) {
        alert(`Tìm thấy ${data.detections.length} người`);
      } else {
        alert("Detection returned no data");
      }

      // Cập nhật danh sách sự kiện
      fetchEvents();
    } catch (error) {
      console.error("Detection failed:", error);
      alert("Detection failed: " + (error as Error).message);
    } finally {
      setLoading(false);
      // Reset input để có thể upload lại cùng file nếu muốn
      if (fileInputRef.current) fileInputRef.current.value = "";
    }
  };

  const handleCreateROI = async () => {
    const roiData = {
      x: 100,
      y: 100,
      width: 400,
      height: 300,
      name: `ROI ${rois.length + 1}`,
    }

    try {
      await apiCall("/roi", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(roiData),
      })
      alert("ROI created successfully!")
      fetchROIs()
    } catch (error) {
      console.error("Failed to create ROI:", error)
      alert("Failed to create ROI: " + (error as Error).message)
    }
  }

  const handleDeleteROI = async (roiId: string) => {
    if (!confirm("Are you sure you want to delete this ROI?")) return

    try {
      await apiCall(`/roi/${roiId}`, { method: "DELETE" })
      alert("ROI deleted successfully!")
      fetchROIs()
    } catch (error) {
      console.error("Failed to delete ROI:", error)
      alert("Failed to delete ROI: " + (error as Error).message)
    }
  }

  const downloadImage = async (url: string, filename: string) => {
    try {
      const response = await fetch(url)
      const blob = await response.blob()
      const link = document.createElement("a")
      link.href = URL.createObjectURL(blob)
      link.download = filename || "detection.jpg"
      link.click()
    } catch (error) {
      console.error("Download failed:", error)
      alert("Failed to download image")
    }
  }

  // Hàm mở cửa
  const handleOpenDoor = async () => {
    if (doorLoading) return;

    try {
      setDoorLoading(true);
      const data = await apiCall("/door/open", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      });

      if (data.success) {
        alert("🚪 " + data.message);
      } else {
        alert("❌ Không thể mở cửa");
      }
    } catch (error) {
      console.error("Open door failed:", error);
      alert("❌ Lỗi: " + (error as Error).message);
    } finally {
      setDoorLoading(false);
    }
  };

  useEffect(() => {
    fetchEvents()
    fetchROIs()
  }, [alertsOnly])

  const DashboardView = () => (
    <div className="space-y-6">
      {/* Stats Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        <StatCard icon={<Activity className="w-6 h-6" />} title="Tổng sự kiện " value={stats.total} color="blue" />
        <StatCard icon={<AlertCircle className="w-6 h-6" />} title="Cảnh báo " value={stats.alerts} color="red" />
        <StatCard icon={<Users className="w-6 h-6" />} title="Bình thường" value={stats.known} color="green" />
        {/* <StatCard icon={<Clock className="w-6 h-6" />} title="Today" value={stats.today} color="purple" /> */}
      </div>

      {/* Main Content Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Live Camera Feed - Left Side (2 columns) */}
        <div className="lg:col-span-2">
          <div className="bg-slate-800 rounded-xl shadow-2xl p-6 border border-slate-700">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-xl font-bold text-white flex items-center gap-2">
                <Video className="w-5 h-5 text-blue-400" />
                 Cam  
              </h2>
              <div className="flex items-center gap-3">
                {isStreamActive ? (
                  <>
                    <div className="w-3 h-3 bg-red-500 rounded-full animate-pulse"></div>
                    <span className="text-sm text-red-400 font-medium">Trực tiếp </span>
                  </>
                ) : (
                  <>
                    <div className="w-3 h-3 bg-slate-500 rounded-full"></div>
                    <span className="text-sm text-slate-400">Offline</span>
                  </>
                )}
              </div>
            </div>

            <div className="relative bg-black rounded-lg overflow-hidden aspect-video group">
              {liveStream ? (
                <>
                  <img
                    src={liveStream || "/placeholder.svg"}
                    alt="Live Feed"
                    className="w-full h-full object-contain"
                  />
                  <div className="absolute inset-0 bg-linear-to-t from-black/60 via-transparent to-transparent opacity-0 group-hover:opacity-100 transition-opacity">
                    <div className="absolute bottom-4 right-4 flex gap-2">
                      <button
                        onClick={() => setSelectedImage(liveStream)}
                        className="bg-white/20! hover:bg-white/30! backdrop-blur-sm! text-white p-2 rounded-lg transition-colors"
                      >
                        <Maximize2 className="w-5 h-5" />
                      </button>
                      <button
                        onClick={() => downloadImage(liveStream, "live-capture.jpg")}
                        className="bg-white/20! hover:bg-white/30 backdrop-blur-sm text-white p-2 rounded-lg transition-colors"
                      >
                        <Download className="w-5 h-5" />
                      </button>
                    </div>
                  </div>
                </>
              ) : (
                <div className="flex flex-col items-center justify-center h-full text-slate-500">
                  <Camera className="w-16 h-16 mb-3 opacity-50" />
                  <p className="text-lg">...</p>
                </div>
              )}
            </div>

            {/* Live Detection Indicators */}
            {liveDetections.length > 0 && (
              <div className="mt-4 space-y-2">
                {liveDetections[0]?.detections?.map((det, idx) => (
                  <div
                    key={idx}
                    className="flex items-center justify-between bg-slate-700/50 rounded-lg p-3 border border-slate-600"
                  >
                    <div className="flex items-center gap-3">
                      <div
                        className={`w-3 h-3 rounded-full ${det.alert ? "bg-red-500 animate-pulse" : "bg-green-500"}`}
                      ></div>
                      <span className="text-white font-medium">
                        {det.alert ? "⚠️ Người lạ" : "✅ Người quen"}
                      </span>
                    </div>
                    <span className="text-slate-400 text-sm">Độ chính xác: {(det.confidence * 100).toFixed(1)}%</span>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>

        {/* Right Sidebar - Notifications & Upload */}
        <div className="space-y-6">
          {/* Recent Detections */}
          <div className="bg-slate-800 rounded-xl shadow-lg p-6 border border-slate-700">
            <h2 className="text-lg font-bold text-white mb-4 flex items-center gap-2">
              <Bell className="w-5 h-5 text-blue-400" />
                Thông báo vừa qua 
            </h2>

            {liveDetections.length === 0 ? (
              <div className="text-center py-8 text-slate-400">
                <AlertCircle className="w-12 h-12 mx-auto mb-2 opacity-50" />
                <p className="text-sm">Chưa có </p>
              </div>
            ) : (
              <div className="space-y-3 max-h-64 overflow-y-auto">
                {liveDetections.map((detection, idx) => (
                  <div
                    key={idx}
                    className={`border-l-4 p-3 rounded-lg cursor-pointer hover:bg-slate-700/50 transition-colors ${detection.alert ? "border-red-500 bg-red-950/20" : "border-green-500 bg-green-950/20"
                      }`}
                    onClick={() => detection.image_url && setSelectedImage(detection.image_url)}
                  >
                    <div className="flex justify-between items-start mb-1">
                      <p className={`font-semibold text-sm ${detection.alert ? "text-red-400" : "text-green-400"}`}>
                        {detection.alert ? "⚠️ Cảnh báo" : "✅ Bình thường"}
                      </p>
                      <span className="text-xs text-slate-400">
                        {new Date(detection.timestamp).toLocaleTimeString()}
                      </span>
                    </div>
                    <p className="text-xs text-slate-400">{detection.detections?.length || 0} người</p>
                  </div>
                ))}
              </div>
            )}
          </div>

          {/* Door Control Section */}
          <div className="bg-slate-800 rounded-xl shadow-lg p-6 border border-slate-700">
            <h2 className="text-lg font-bold text-white mb-4 flex items-center gap-2">
              <DoorOpen className="w-5 h-5 text-green-400" />
              Điều khiển cửa
            </h2>
            <div className="space-y-4">
              
              <button
                onClick={handleOpenDoor}
                disabled={doorLoading}
                className="w-full bg-gradient-to-r from-green-600 to-emerald-600 hover:from-green-500 hover:to-emerald-500 disabled:from-gray-600 disabled:to-gray-600 text-white py-4 rounded-xl font-bold text-lg shadow-lg shadow-green-500/25 transition-all duration-300 flex items-center justify-center gap-3 disabled:opacity-50"
              >
                {doorLoading ? (
                  <>
                    <Loader2 className="w-6 h-6 animate-spin" />
                    Đang gửi lệnh...
                  </>
                ) : (
                  <>
                    <DoorOpen className="w-6 h-6" />
                    🔓 Mở cửa
                  </>
                )}
              </button>
              <p className="text-xs text-slate-500 text-center">
                Cửa sẽ tự động đóng sau 4 giây
              </p>
            </div>
          </div>

          {/* Upload Section */}
          <div className="bg-slate-800 rounded-xl shadow-lg p-6 border border-slate-700">
            <h2 className="text-lg font-bold text-white mb-4 flex items-center gap-2">
              <Upload className="w-5 h-5 text-blue-400" />
              Kiểm tra phát hiện 
            </h2>
            <div
              className="border-2 border-dashed border-slate-600 rounded-lg p-6 text-center hover:border-blue-400 transition-colors cursor-pointer"
              onClick={() => fileInputRef.current?.click()}
            >
              <input ref={fileInputRef} type="file" accept="image/*" onChange={handleImageUpload} className="hidden" />
              <Camera className="w-12 h-12 mx-auto mb-3 text-slate-500" />
              <button
                disabled={loading}
                className="bg-blue-600! hover:bg-blue-700! text-white px-4 py-2 rounded-lg font-medium disabled:opacity-50 transition-colors text-sm"
              >
                {loading ? "Đang xử lý..." : "Tải ảnh lên"}
              </button>
              <p className="text-xs text-slate-400 mt-2">JPG, PNG (tối đa 10MB)</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  )

  const EventsView = () => (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h2 className="text-2xl text-white font-bold">Lịch sử </h2>
        <div className="flex items-center gap-4">
          <label className="flex items-center gap-2 cursor-pointer  text-white">
            <input
              type="checkbox"
              checked={alertsOnly}
              onChange={(e) => {setAlertsOnly(e.target.checked)}}
              className="w-4 h-4 text-red-600 rounded"
            />
            <span className="text-sm ">Chỉ cảnh báo </span>
          </label>
          <button
            onClick={fetchEvents}
            disabled={loading}
            className="bg-blue-600! hover:bg-blue-700! text-white px-4 py-2 rounded-lg text-sm font-medium transition-colors disabled:opacity-50"
          >
            Làm mới 
          </button>
        </div>
      </div>

      {loading && events.length === 0 ? (
        <div className="text-center py-12">
          <div className="animate-spin w-12 h-12 border-4 border-blue-600 border-t-transparent rounded-full mx-auto"></div>
          <p className="mt-4 text-slate-400">Loading events...</p>
        </div>
      ) : events.length === 0 ? (
        <div className="bg-slate-800 rounded-lg shadow-md p-12 text-center border border-slate-700">
          <AlertCircle className="w-16 h-16 mx-auto mb-4 text-slate-500" />
          <p className="text-slate-400">No events found</p>
        </div>
      ) : (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {events.map((event) => (
            <EventCard
              key={event.event_id || event.id}
              event={event}
              onViewImage={setSelectedImage}
              onDownload={downloadImage}
            />
          ))}
        </div>
      )}
    </div>
  )

  const ROIView = () => (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h2 className="text-2xl font-bold">Region of Interest (ROI)</h2>
        <button
          onClick={handleCreateROI}
          className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg font-medium flex items-center gap-2 transition-colors"
        >
          <MapPin className="w-4 h-4" />
          Create ROI
        </button>
      </div>

      {rois.length > 0 ? (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {rois.map((roi: any) => (
            <div
              key={roi.roi_id}
              className="bg-slate-800 rounded-lg shadow-md p-6 hover:shadow-lg transition-shadow border border-slate-700"
            >
              <div className="flex justify-between items-start mb-4">
                <div>
                  <h3 className="font-bold text-lg text-white">{roi.roi.name}</h3>
                  <span
                    className={`text-xs px-2 py-1 rounded-full ${roi.active ? "bg-green-900/50 text-green-400" : "bg-slate-700 text-slate-400"
                      }`}
                  >
                    {roi.active ? "Active" : "Inactive"}
                  </span>
                </div>
                <button
                  onClick={() => handleDeleteROI(roi.roi_id)}
                  className="text-red-600 hover:text-red-400 transition-colors"
                >
                  <Trash2 className="w-5 h-5" />
                </button>
              </div>
              <div className="space-y-2 text-sm text-slate-400">
                <div className="flex justify-between">
                  <span>Position:</span>
                  <span className="font-mono">
                    ({roi.roi.x}, {roi.roi.y})
                  </span>
                </div>
                <div className="flex justify-between">
                  <span>Size:</span>
                  <span className="font-mono">
                    {roi.roi.width} × {roi.roi.height}
                  </span>
                </div>
              </div>
            </div>
          ))}
        </div>
      ) : (
        <div className="bg-slate-800 rounded-lg shadow-md p-12 text-center border border-slate-700">
          <MapPin className="w-16 h-16 mx-auto mb-4 text-slate-500" />
          <p className="text-slate-400 mb-4">No ROIs configured</p>
          <button
            onClick={handleCreateROI}
            className="bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded-lg font-medium transition-colors"
          >
            Create First ROI
          </button>
        </div>
      )}
    </div>
  )

  const SettingsView = () => (
    <div className="space-y-6">
      <h2 className="text-2xl font-bold">Settings</h2>

      <div className="bg-slate-800 rounded-xl shadow-lg p-6 border border-slate-700">
        <h3 className="text-lg font-bold text-white mb-4">Notification Settings</h3>
        <label className="flex items-center justify-between cursor-pointer group">
          <div className="flex items-center gap-3">
            <div className={`p-2 rounded-lg ${notificationsEnabled ? "bg-blue-900/50" : "bg-slate-700"}`}>
              <Bell className={`w-5 h-5 ${notificationsEnabled ? "text-blue-400" : "text-slate-500"}`} />
            </div>
            <div>
              <p className="font-medium text-white">Enable Notifications</p>
              <p className="text-sm text-slate-400">Receive alerts when intrusion is detected</p>
            </div>
          </div>
          <div className="relative">
            <input
              type="checkbox"
              checked={notificationsEnabled}
              onChange={(e) => setNotificationsEnabled(e.target.checked)}
              className="sr-only"
            />
            <div
              className={`w-14 h-8 rounded-full transition-colors ${notificationsEnabled ? "bg-blue-600" : "bg-slate-600"
                }`}
            >
              <div
                className={`w-6 h-6 bg-white rounded-full shadow-md transform transition-transform duration-200 ease-in-out mt-1 ${notificationsEnabled ? "translate-x-7" : "translate-x-1"
                  }`}
              ></div>
            </div>
          </div>
        </label>
      </div>

      <div className="bg-slate-800 rounded-xl shadow-lg p-6 border border-slate-700">
        <h3 className="text-lg font-bold text-white mb-4">System Configuration</h3>
        <div className="space-y-4">
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">API Base URL</label>
            <input
              type="text"
              value={API_BASE_URL}
              readOnly
              className="w-full px-4 py-2 border border-slate-600 rounded-lg bg-slate-700/50 font-mono text-sm text-slate-300"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">WebSocket URL</label>
            <input
              type="text"
              value={WS_URL}
              readOnly
              className="w-full px-4 py-2 border border-slate-600 rounded-lg bg-slate-700/50 font-mono text-sm text-slate-300"
            />
          </div>
        </div>
      </div>
    </div>
  )

  return (
    <div className="min-h-screen min-w-screen bg-linear-to-br from-slate-900 via-slate-800 to-slate-900">
      {/* Alert Sound */}
      <audio
        ref={audioRef}
        src="data:audio/wav;base64,UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIGWi88OScTgwNUKXh8bllHAU2jdXxx3ElBSF1ye/glEILElyx6OyrWBUIQ5zd8sFuJAUuhM/z1YU4CRRoufLnnFIKDEyk4PK2ZRwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwEPYvR8Mt+LgYef8/x4JZDCxJcr+jqrlsVB0OY3PC/cSYFKn/O8daLOQgYZbzvn1QKC0mi4PK2ZhwE"
      />

      {/* Floating Notifications */}
      <div className="fixed top-4 right-4 z-50 space-y-3 max-w-sm">
        {notifications.map((notif) => (
          <NotificationToast
            key={notif.id}
            notification={notif}
            onClose={() => setNotifications((prev) => prev.filter((n) => n.id !== notif.id))}
            onView={() => {
              notif.image_url && setSelectedImage(notif.image_url)
            }}
          />
        ))}
      </div>

      {/* Image Modal */}
      {selectedImage && (
        <ImageModal
          imageUrl={selectedImage}
          onClose={() => setSelectedImage(null)}
          onDownload={() => downloadImage(selectedImage, "detection-image.jpg")}
        />
      )}

      {/* Header */}
      <header className="bg-linear-to-r from-slate-900 to-slate-800 shadow-xl border-b border-slate-700">
        <div className="max-w-7xl mx-auto px-4 py-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center">
            <div className="flex items-center gap-3">
              <div className="bg-blue-600 p-2 rounded-lg shadow-lg">
                <Camera className="w-6 h-6 text-white" />
              </div>
              <div>
                <h1 className="text-2xl font-bold text-white">Hệ thống phát hiện đột nhập </h1>
              </div>
            </div>
            <div className="flex items-center gap-4">
              
            </div>
          </div>
        </div>
      </header>

      {/* Navigation */}
      <nav className="bg-slate-800/80 border-b border-slate-700 shadow-sm backdrop-blur-sm sticky top-0 z-40">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex gap-8">
            <NavTab
              icon={<Activity className="w-4 h-4" />}
              label="Màn hình chính"
              active={activeTab === "dashboard"}
              onClick={() => setActiveTab("dashboard")}
            />
            <NavTab
              icon={<Bell className="w-4 h-4" />}
              label="Sự kiện "
              active={activeTab === "events"}
              onClick={() => setActiveTab("events")}
              // badge={stats.alerts}
            />
            {/* <NavTab
              icon={<MapPin className="w-4 h-4" />}
              label="ROI"
              active={activeTab === "roi"}
              onClick={() => setActiveTab("roi")}
            /> */}
            {/* <NavTab
              icon={<Settings className="w-4 h-4" />}
              label="Settings"
              active={activeTab === "settings"}
              onClick={() => setActiveTab("settings")}
            /> */}
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 py-8 sm:px-6 lg:px-8">
        {activeTab === "dashboard" && <DashboardView />}
        {activeTab === "events" && <EventsView />}
        {activeTab === "roi" && <ROIView />}
        {activeTab === "settings" && <SettingsView />}
      </main>
    </div>
  )
}

// Component: Stat Card
const StatCard = ({ icon, title, value, color }: any) => {
  const colors: Record<string, string> = {
    blue: "from-blue-500 to-blue-600 shadow-blue-500/30",
    red: "from-red-500 to-red-600 shadow-red-500/30",
    green: "from-green-500 to-green-600 shadow-green-500/30",
    purple: "from-purple-500 to-purple-600 shadow-purple-500/30",
  }

  return (
    <div className="bg-slate-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300 border border-slate-700">
      <div className="flex items-center justify-between">
        <div>
          <p className="text-sm text-slate-400 mb-1 font-medium">{title}</p>
          <p className="text-3xl font-bold text-white">{value}</p>
        </div>
        <div className={`p-4 rounded-xl bg-linear-to-br ${colors[color]} shadow-lg text-white`}>{icon}</div>
      </div>
    </div>
  )
}

// Component: Nav Tab
const NavTab = ({ icon, label, active, onClick, badge }: any) => (
  <button
    onClick={onClick}
    className={`flex items-center gap-2 px-1 py-4 border-b-2 font-medium text-sm transition-all relative ${active
        ? "border-blue-500 text-blue-400"
        : "border-transparent! text-slate-400! hover:text-red-300! hover:border-slate-600!"
      }`}
  >
    {icon}
    {label}
    {badge > 0 && (
      <span className="absolute -top-1 -right-2 bg-red-500 text-white text-xs w-5 h-5 rounded-full flex items-center justify-center animate-pulse shadow-lg">
        {badge}
      </span>
    )}
  </button>
)

// Component: Event Card
const EventCard = ({ event, onViewImage, onDownload }: any) => (
  <div
    className={`bg-slate-800 rounded-xl shadow-md hover:shadow-xl transition-all duration-300 overflow-hidden border-l-4 border-slate-700 ${event.alert ? "border-l-red-500" : "border-l-green-500"
      }`}
  >
    {event.image_url && (
      <div className="relative aspect-video bg-black group cursor-pointer" onClick={() => onViewImage(event.image_url)}>
        <img src={event.image_url || "/placeholder.svg"} alt="Detection" className="w-full h-full object-cover" />
        <div className="absolute inset-0 bg-black/40 opacity-0 group-hover:opacity-100 transition-opacity flex items-center justify-center gap-2">
          <button className="bg-white/90 hover:bg-white p-2 rounded-lg transition-colors">
            <Eye className="w-5 h-5 text-slate-900" />
          </button>
          <button
            onClick={(e) => {
              e.stopPropagation()
              onDownload(event.image_url, `detection-${event.event_id || event.id}.jpg`)
            }}
            className="bg-white/90 hover:bg-white p-2 rounded-lg transition-colors"
          >
            <Download className="w-5 h-5 text-slate-900" />
          </button>
        </div>
      </div>
    )}
    <div className="p-5">
      <div className="flex items-start justify-between mb-3">
        <div className="flex items-center gap-2">
          {event.alert ? (
            <AlertCircle className="w-5 h-5 text-red-500" />
          ) : (
            <CheckCircle className="w-5 h-5 text-green-500" />
          )}
          <h3 className="font-bold text-white">{event.alert ? "Người lạ" : "Người Quen"}</h3>
        </div>
      </div>
      <div className="space-y-2 text-sm text-slate-400 mb-3">
        <p className="flex items-center gap-2">
          <Clock className="w-4 h-4" />
          {new Date(event.timestamp).toLocaleString()}
        </p>
        <p className="flex items-center gap-2">
          <Users className="w-4 h-4" />
          {event.detections?.length || 0} detection(s)
        </p>
      </div>
      <div className="flex flex-wrap gap-2">
        {event.detections?.slice(0, 2).map((det: any, idx: number) => (
          <span
            key={idx}
            className={`text-xs px-3 py-1 rounded-full font-medium ${det.alert ? "bg-red-900/50 text-red-300" : "bg-green-900/50 text-green-300"
              }`}
          >
            {det.face_id} ({(det.confidence * 100).toFixed(0)}%)
          </span>
        ))}
      </div>
    </div>
  </div>
)

// Component: Notification Toast
const NotificationToast = ({ notification, onClose, onView }: any) => (
  <div
    className={`animate-slide-in-right bg-slate-800 rounded-lg shadow-2xl border-l-4 overflow-hidden max-w-sm border-slate-700 ${notification.alert ? "border-l-red-500" : "border-l-green-500"
      } ${notification.read ? "opacity-75" : ""}`}
  >
    <div className="p-4">
      <div className="flex items-start justify-between mb-2">
        <div className="flex items-center gap-2">
          {notification.alert ? (
            <div className="bg-red-900/50 p-2 rounded-lg">
              <AlertCircle className="w-5 h-5 text-red-500 animate-pulse" />
            </div>
          ) : (
            <div className="bg-green-900/50 p-2 rounded-lg">
              <Users className="w-5 h-5 text-green-500" />
            </div>
          )}
          <div>
            <p className={`font-bold text-sm ${notification.alert ? "text-red-400!" : "text-green-400!"}`}>
              {notification.alert ? "⚠️ Cảnh báo có người đột nhập!" : " Phát hiện có người"}
            </p>
            <p className="text-xs text-slate-400">{new Date(notification.timestamp).toLocaleTimeString()}</p>
          </div>
        </div>
        <button onClick={onClose} className="text-slate-400 hover:text-slate-200 transition-colors">
          <X className="w-4 h-4" />
        </button>
      </div>

      {notification.image_url && (
        <img
          src={notification.image_url || "/placeholder.svg"}
          alt="Detection"
          className="w-full h-32 object-cover rounded-lg mb-3 cursor-pointer hover:opacity-90 transition-opacity"
          onClick={onView}
        />
      )}

      <p className="text-sm text-slate-400! mb-3">{notification.detections?.length || 0} người được phát hiện</p>

      <button
        onClick={onView}
        className={`w-full py-2 rounded-lg font-medium text-sm transition-colors ${notification.alert ? "bg-red-600! hover:bg-red-700! text-white" : "bg-green-600! hover:bg-green-700! text-white"
          }`}
      >
        View Details
      </button>
    </div>
  </div>
)

// Component: Image Modal
const ImageModal = ({ imageUrl, onClose, onDownload }: any) => (
  <div
    className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/80 backdrop-blur-sm animate-fade-scale"
    onClick={onClose}
  >
    <div className="relative max-w-4xl w-full" onClick={(e) => e.stopPropagation()}>
      <div className="bg-slate-800 rounded-2xl shadow-2xl overflow-hidden border border-slate-700">
        <div className="bg-slate-900 p-4 flex items-center justify-between">
          <h3 className="text-white font-bold">Detection Image</h3>
          <div className="flex gap-2">
            <button
              onClick={onDownload}
              className="bg-blue-600! hover:bg-blue-700! text-white p-2 rounded-lg transition-colors"
            >
              <Download className="w-5 h-5" />
            </button>
            <button
              onClick={onClose}
              className="bg-slate-700! hover:bg-slate-600! text-white p-2 rounded-lg transition-colors"
            >
              <X className="w-5 h-5" />
            </button>
          </div>
        </div>
        <img
          src={imageUrl || "/placeholder.svg"}
          alt="Full size"
          className="w-full max-h-[70vh] object-contain bg-black"
        />
      </div>
    </div>
  </div>
)

export default App
