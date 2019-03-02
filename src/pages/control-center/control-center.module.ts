import { NgModule } from '@angular/core';
import { IonicPageModule } from 'ionic-angular';
import { ControlCenterPage } from './control-center';

@NgModule({
  declarations: [
    ControlCenterPage,
  ],
  imports: [
    IonicPageModule.forChild(ControlCenterPage),
  ],
})
export class ControlCenterPageModule {}
