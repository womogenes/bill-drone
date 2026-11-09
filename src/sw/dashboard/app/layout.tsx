import { Inter } from 'next/font/google';
import type { Metadata } from 'next';

import './globals.css';
import { cn } from '@/lib/utils';

const inter = Inter({ subsets: ['latin'], variable: '--font-sans' });

export const metadata: Metadata = {
  title: 'Drone dashboard',
  description: 'Drone sensor data here',
  icons: {
    icon: 'https://fav.farm/✈️',
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className={cn('font-sans antialiased', inter.variable)}>
      <body>{children}</body>
    </html>
  );
}
