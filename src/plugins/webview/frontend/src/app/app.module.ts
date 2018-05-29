
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { ServiceWorkerModule } from '@angular/service-worker';
import { HttpClientModule } from '@angular/common/http';
import { WebStorageModule } from 'ngx-store';

import { environment } from '../environments/environment';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { ChromeModule } from '../chrome/module';
import { BackendConfigurationService } from '../services/backend-config/backend-config.service';
import { ConfigurationService } from '../services/config/config.service';
import { ConfigurationApiService } from '../services/config/api.service';
import { AssetsService } from '../services/global/assets';
import { SwUpdateNotifierService } from '../services/update-notifier/update-notifier.service';
import { LockoutService } from '../services/lockout/lockout.service';

import { DashboardModule } from '../parts/dashboard/module';
import { ClipsExecutiveModule } from '../parts/clips-executive/module';
import { SkillerModule } from '../parts/skiller/module';
import { BlackboardModule } from '../parts/blackboard/module';
import { ClipsModule } from '../parts/clips/module';
import { ImageModule } from '../parts/images/module';
import { PluginModule } from '../parts/plugins/module';
import { TransformsModule } from '../parts/transforms/module';

@NgModule({
	declarations: [
	  AppComponent,
	],
	imports: [
	  BrowserModule,
	  BrowserAnimationsModule,
	  
	  HttpClientModule,
	  ServiceWorkerModule.register('/ngsw-worker.js', { enabled: environment.production }),
	  WebStorageModule,

	  ChromeModule,
	  DashboardModule,
	  ClipsExecutiveModule,
	  SkillerModule,
	  BlackboardModule,
	  ClipsModule,
	  ImageModule,
	  PluginModule,
	  TransformsModule,

	  // Keep the AppRoutingModule last
	  AppRoutingModule,
	],
  providers: [LockoutService, BackendConfigurationService,
              ConfigurationService, ConfigurationApiService,
              AssetsService, SwUpdateNotifierService],
	bootstrap: [AppComponent]
})
export class AppModule { }
