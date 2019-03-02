import { BrowserModule } from '@angular/platform-browser';
import { ErrorHandler, NgModule } from '@angular/core';
import { IonicApp, IonicErrorHandler, IonicModule } from 'ionic-angular';
import { MyApp } from './app.component';
import { HomePage } from '../pages/home/home';
import { StatusBar } from '@ionic-native/status-bar';
import { SplashScreen } from '@ionic-native/splash-screen';
import { WelcomePage } from '../pages/welcome/welcome';
import { HomePageModule } from '../pages/home/home.module';
import { ControlCenterPage } from '../pages/control-center/control-center';
import { StatisticsPage } from '../pages/statistics/statistics';
import { BluetoothSerial } from '@ionic-native/bluetooth-serial';
import { ShowDevicesPage } from '../pages/show-devices/show-devices';
import { GoogleMaps } from '@ionic-native/google-maps';
import { SpeechRecognition } from '@ionic-native/speech-recognition';
import { SMS } from '@ionic-native/sms';
import { DeviceOrientation} from '@ionic-native/device-orientation';


@NgModule({
  declarations: [
    MyApp,
    WelcomePage,
    ControlCenterPage,
    StatisticsPage,
    ShowDevicesPage,
  ],
  imports: [
    BrowserModule,
    HomePageModule,
    IonicModule.forRoot(MyApp),
  ],
  bootstrap: [IonicApp],
  entryComponents: [
    MyApp,
    HomePage,
    WelcomePage,
    ControlCenterPage,
    StatisticsPage,
    ShowDevicesPage,
  ],
  providers: [
    StatusBar,
    SplashScreen,
    BluetoothSerial,
    GoogleMaps,
    SpeechRecognition,
    SMS,
    DeviceOrientation,
    {provide: ErrorHandler, useClass: IonicErrorHandler}
  ]
})
export class AppModule {}
