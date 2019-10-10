import React, { useEffect, useRef } from "react";
import { basicBlock } from "../styleDefs/styles";
import * as ROSLIB from "roslib";
import { Helmet } from "react-helmet";
import adapter from "webrtc-adapter";

declare var WebrtcRos: any;

interface VidsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  ipAddr: string;
  ipPort: string;
}

// Takes a parameter ros, which is the connection to ros
const Vids: React.FunctionComponent<VidsProps> = ({
  ros,
  connected,
  ipAddr,
  ipPort
}) => {
  const remoteRefUpper = React.useRef(null);
  const remoteRefLower = React.useRef(null);
  const localRef = React.useRef(null);

  useEffect(() => {
    if (connected) {
      const connection1 = WebrtcRos.createConnection(
        (window.location.protocol === "https:" ? "wss://" : "ws://") +
          ipAddr +
          ":" +
          (parseInt(ipPort) + 1) +
          "/webrtc"
      );

      connection1.onConfigurationNeeded = () => {
        const remote_stream_config_upper = { video: {}, audio: {} };
        remote_stream_config_upper.video = {
          id: "subscribed_video_upper",
            src: "ros_image:/upper_realsense/color/image_raw",
        };
        remote_stream_config_upper.audio = {
          id: "subscribed_audio",
          src: "local:"
        };

        connection1
          .addRemoteStream(remote_stream_config_upper)
          .then((event: any) => {
            //stream started
            let remoteVideoElement = remoteRefUpper as any;
            remoteVideoElement.current.srcObject = event.stream;
            event.remove.then(function(event: any) {
              //Remote stream removed
              remoteVideoElement.srcObject = null;
            });
            //(window as any).remotestream = event.stream;
          });

        let user_media_config = { video: {}, audio: {} };
        let local_stream_config = { video: {}, audio: {} };
        user_media_config.video = true;
        local_stream_config.video = {
          dest: "remote_video"
        };
        user_media_config.audio = true;

        connection1
          .addLocalStream(user_media_config, local_stream_config)
          .then(function(event: any) {
            console.log(
              "Local stream added",
              event,
              event.stream.getVideoTracks(),
              event.stream.getAudioTracks()
            );
            let localVideoElement = localRef as any;
            localVideoElement.current.srcObject = event.stream;
            event.remove.then(function(event: any) {
              //console.log("Local stream removed", event);
              localVideoElement.current.srcObject = null;
            });
            //window.localstream = event.stream;
          });

        connection1.sendConfigure();
      };
      connection1.connect();

      const connection2 = WebrtcRos.createConnection(
        (window.location.protocol === "https:" ? "wss://" : "ws://") +
          ipAddr +
          ":" +
          (parseInt(ipPort) + 1) +
          "/webrtc"
      );

      connection2.onConfigurationNeeded = () => {
        const remote_stream_config_lower = { video: {}, audio: {} };
        remote_stream_config_lower.video = {
          id: "subscribed_video_lower",
          src: "ros_image:/lower_realsense/color/image_raw"
        };

        connection2
          .addRemoteStream(remote_stream_config_lower)
          .then((event: any) => {
            //stream started
            let remoteVideoElement = remoteRefLower as any;
            remoteVideoElement.current.srcObject = event.stream;
            event.remove.then(function(event: any) {
              //Remote stream removed
              remoteVideoElement.srcObject = null;
            });
            //(window as any).remotestream = event.stream;
          });
        connection2.sendConfigure();
      };
      connection2.connect();
    }
  }, [connected, ros]);
  //<script type="text/javascript" src={"/web/adapter.js"} />
  //

  const vidStyle = {
    width: "auto",
    maxWidth: "300px"
  };

  return (
      <>
      <Helmet>
        <script type="text/javascript" src={"/web/webrtc_ros.js"} />
      </Helmet>,
        <video
          ref={remoteRefUpper}
          id="remote-video-upper"
          autoPlay={true}
          style={vidStyle}
      ></video>,
        <video
          ref={remoteRefLower}
          id="remote-video-lower"
          autoPlay={true}
          style={vidStyle}
        ></video>,
        <video
          ref={localRef}
          id="local-video"
          autoPlay={true}
          style={vidStyle}
          muted={true}
        ></video>
    </>
  );
};

export default Vids;
