import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:fluttertoast/fluttertoast.dart';

class HomePage extends StatefulWidget {
  const HomePage({Key? key}) : super(key: key);

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  FlutterBluePlus flutterBlue = FlutterBluePlus.instance;
  final availableDevices = <BluetoothDevice>[];
  BluetoothDevice? anemometer = null;
  BluetoothService? windSpeedService = null;
  BluetoothCharacteristic? windSpeedCharcteristic = null;
  Timer? periodicTimerHandle = null;
  double windSpeed = 0;

  @override
  void initState() {
    // TODO: implement initState
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text("Anemometer"),
        ),
        body: FutureBuilder(
            future: bodyWidget(),
            builder: (context, AsyncSnapshot<Widget> snapshot) {
              if (snapshot.hasData) {
                return snapshot.data!;
              }
              return const Center(child: Text("Shomething went wrong"));
            }));
  }

  Widget connectToADeviceWidget() {
    return Center(
        child: RefreshIndicator(
            onRefresh: startDiscovery,
            child: ListView.builder(
              itemBuilder: ((context, index) => ListTile(
                    title: Text(availableDevices[index].name),
                    onTap: () => connectToDevice(index),
                  )),
              itemCount: availableDevices.length,
            )));
  }

  Future<void> connectToDevice(int deviceIndex) async {
    await availableDevices[deviceIndex].connect();
    List<BluetoothDevice> connectedDevices = await flutterBlue.connectedDevices;
    if (connectedDevices.contains(availableDevices[deviceIndex])) {
      List<BluetoothService> services =
          await availableDevices[deviceIndex].discoverServices();
      services.forEach((service) {
        if (service.isPrimary) {
          windSpeedService = service;
          for (BluetoothCharacteristic characteristic
              in service.characteristics) {
            print("UUID: ${characteristic.serviceUuid}");
            if (characteristic.serviceUuid.toString() ==
                "00000000-0000-0000-0000-00000000702a") {
              windSpeedCharcteristic = characteristic;
            }
          }
        }
      });
      setState(() {
        anemometer = availableDevices[deviceIndex];
      });
    }
    Fluttertoast.showToast(
        msg: "Connection successful!",
        toastLength: Toast.LENGTH_SHORT,
        backgroundColor: Colors.green);
    if (periodicTimerHandle == null) {
      periodicTimerHandle = Timer.periodic(Duration(seconds: 1), (timer) async {
        await fetchWindSpeed();
      });
    }
  }

  Future<void> startDiscovery() async {
    setState(() => availableDevices.removeWhere((element) => true));
    await flutterBlue.startScan(timeout: Duration(seconds: 4));
    flutterBlue.scanResults.listen((result) {
      for (var r in result) {
        if (!availableDevices.contains(r.device) &&
            r.device.name == "Anemometer V1") {
          print("FOUND Anemometer: ${r.device.name}");
          setState(() => availableDevices.add(r.device));
        }
      }
    });
    flutterBlue.stopScan();
  }

  Widget turnOnBtWidget() {
    return RefreshIndicator(
      child: Center(child: Text("Turn on bluetooth")),
      onRefresh: () async => (await flutterBlue.isOn) ? setState(() {}) : null,
    );
  }

  Widget windSpeedWidget() {
    return Center(
        child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
          Text("Wind Speed", style: TextStyle(
            fontSize: 26, fontWeight: FontWeight.w300
          )),
          Text("${windSpeed.toStringAsFixed(2)} m/s",
              style: TextStyle(fontSize: 21, fontWeight: FontWeight.bold))
        ]));
  }

  Future<void> fetchWindSpeed() async {
    List<int> readData = await windSpeedCharcteristic!.read();
    setState(() {
      windSpeed =
          ByteData.sublistView(Uint8List.fromList(readData.reversed.toList()))
              .getFloat32(0);
    });
  }

  Future<Widget> bodyWidget() async {
    if (await flutterBlue.isOn) {
      try {
        List<BluetoothDevice> connectedDevices =
            await flutterBlue.connectedDevices;

        if (anemometer == null) {
          connectedDevices
              .firstWhere((element) => element.name == "Anemometer V1")
              .disconnect();
          return connectToADeviceWidget();
        }
      } on StateError catch (_, e) {
        return connectToADeviceWidget();
      }
      if (windSpeedCharcteristic == null) {
        Fluttertoast.showToast(
            msg: "Connection failed",
            toastLength: Toast.LENGTH_SHORT,
            backgroundColor: Colors.red);
        setState(() async {
          await anemometer!.disconnect();
          anemometer = null;
          windSpeedCharcteristic = null;
          windSpeedService = null;
        });
      }
      return windSpeedWidget();
    } else {
      return turnOnBtWidget();
    }
  }
}
